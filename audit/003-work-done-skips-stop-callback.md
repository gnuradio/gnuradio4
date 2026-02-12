# Audit Finding 003: User-Returned DONE Bypasses `stop()` Lifecycle Callback

**Area:** Block lifecycle transition when `processBulk`/`processOne` returns `DONE`
**File:** `core/include/gnuradio-4.0/Block.hpp`, line 2197
**Severity:** Medium (skipped lifecycle callback, potential resource leak)
**Risk of Fix:** Low

---

## Current Behavior

`Block::workInternal()` handles two shutdown paths:

### Path A — EOS tag detected (line 2067–2072):

```cpp
if (isEosTagPresent || lifecycle::isShuttingDown(this->state()) || asyncEoS) {
    emitErrorMessageIfAny("...", this->changeStateTo(lifecycle::State::REQUESTED_STOP));  // ← triggers stop()
    publishEoS();
    this->setAndNotifyState(lifecycle::State::STOPPED);
    return {requestedWork, 0UZ, DONE};
}
```

This path correctly calls `changeStateTo(REQUESTED_STOP)`, which:
1. Validates the transition via `isValidTransition()`
2. Sets state to `REQUESTED_STOP`
3. Invokes the block's `stop()` lifecycle callback (if implemented)

Then `setAndNotifyState(STOPPED)` completes the transition.

### Path B — User processing function returns DONE (line 2195–2199):

```cpp
if (userReturnStatus == DONE) {
    this->setAndNotifyState(lifecycle::State::STOPPED);   // ← bypasses changeStateTo entirely
    publishEoS(outputSpans);
}
```

This path calls `setAndNotifyState(STOPPED)` directly — a protected method
that sets the atomic state and notifies waiters, but:
1. Does **not** call `isValidTransition()` — the transition RUNNING → STOPPED
   is invalid per the state machine (RUNNING can only go to REQUESTED_PAUSE or
   REQUESTED_STOP, per `LifeCycle.hpp:82`)
2. Does **not** invoke the `stop()` lifecycle callback
3. Skips the REQUESTED_STOP intermediate state entirely

## Issue

When a block's `processBulk()` or `processOne()` returns `work::Status::DONE`
(signaling it has finished producing/consuming data), the block transitions
directly from RUNNING to STOPPED. The `stop()` lifecycle callback is never
invoked.

Any block that relies on `stop()` for cleanup — flushing buffers, closing file
handles, releasing hardware resources, sending final messages, updating
metrics — will silently skip that cleanup when the block's own processing
function signals completion.

This is inconsistent: the EOS path (Path A) and `requestStop()` (line 1237)
both go through `changeStateTo(REQUESTED_STOP)`, which invokes `stop()`. Only
the user-DONE path skips it.

The state machine diagram in `LifeCycle.hpp` (lines 31–65) documents RUNNING →
REQUESTED_STOP → STOPPED as the required shutdown sequence. Path B violates
this.

## Proposed Fix

Add the missing `changeStateTo(REQUESTED_STOP)` before the direct state set,
matching the pattern used in the EOS path:

```diff
         // if the block state changed to DONE, publish EOS tag on the next sample
         if (userReturnStatus == DONE) {
+            emitErrorMessageIfAny("workInternal(): processBulk/One returned DONE", this->changeStateTo(lifecycle::State::REQUESTED_STOP));
             this->setAndNotifyState(lifecycle::State::STOPPED);
             publishEoS(outputSpans);
         }
```

One line added. The `changeStateTo(REQUESTED_STOP)` call:
- Validates the transition (RUNNING → REQUESTED_STOP is valid)
- Invokes the block's `stop()` callback if implemented
- Is idempotent if the state is already REQUESTED_STOP (early return at
  `LifeCycle.hpp:189`)

The subsequent `setAndNotifyState(STOPPED)` then completes the sequence
(REQUESTED_STOP → STOPPED), matching the EOS path exactly.

## Why This Helps

- **Correctness:** Blocks that implement `stop()` for resource cleanup will
  have it called regardless of whether shutdown is triggered by EOS or by the
  block's own processing function returning DONE.
- **Consistency:** Both shutdown paths now follow the same state machine
  sequence (RUNNING → REQUESTED_STOP → STOPPED).
- **State machine integrity:** Eliminates an invalid transition that bypasses
  the documented lifecycle diagram.

## Risk Assessment

**Low.** One line added, using the same pattern already present in the EOS
path (line 2068). The `changeStateTo(REQUESTED_STOP)` call is used extensively
throughout the codebase (`requestStop()` at line 1237, `stop()` in
`Scheduler.hpp`, etc.) and is a well-exercised code path. The `stop()` callback
is already called from within `workInternal` in the EOS path, so reentrancy
from `stop()` during `work()` is an accepted pattern.

## Related Observations (not part of this fix)

- The EOS path calls `publishEoS()` **before** `setAndNotifyState(STOPPED)`,
  while the DONE path calls `publishEoS(outputSpans)` **after**. This ordering
  difference may or may not be intentional and could be worth a separate review.
- Both paths use `setAndNotifyState(STOPPED)` to bypass `changeStateTo` for
  the final REQUESTED_STOP → STOPPED transition. This is safe (the transition
  is valid and has no lifecycle callback), but using `changeStateTo(STOPPED)`
  would add the validation check at negligible cost.
