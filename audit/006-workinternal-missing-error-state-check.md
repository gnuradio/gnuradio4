# Audit Finding 006: `workInternal` Processes Data on Block in ERROR State

**Area:** Block work dispatch when lifecycle state is ERROR
**File:** `core/include/gnuradio-4.0/Block.hpp`, lines 2007–2031
**Severity:** Medium (calls processing function on potentially uninitialized block)
**Risk of Fix:** Low

---

## Current Behavior

`Block::workInternal()` performs two lifecycle-state checks before entering the
processing pipeline:

```cpp
work::Result workInternal(std::size_t requestedWork)
{
    using enum gr::work::Status;
    // ...
    applyChangedSettings();

    if constexpr (!blockingIO) {
        if (this->state() == lifecycle::State::REQUESTED_STOP) {           // check 1
            emitErrorMessageIfAny("...", this->changeStateTo(lifecycle::State::STOPPED));
        }
    }

    // ...

    if (this->state() == lifecycle::State::STOPPED) {                      // check 2
        disconnectFromUpStreamParents();
        return {requestedWork, 0UZ, DONE};
    }

    // --- full processing pipeline follows ---
    // prepareStreams, updateMergedInputTagAndApplySettings, publishCachedOutputTags,
    // invokeProcessBulk / invokeProcessOne, consumeReaders, publishWriters, ...
}
```

There is no check for `lifecycle::State::ERROR`.

## Issue

A block can enter ERROR state through several paths:

1. **Failed `start()` callback** — `invokeLifecycleMethod` (LifeCycle.hpp:143–157)
   catches exceptions from `start()` and calls `setAndNotifyState(State::ERROR)`.
   The scheduler's `start()` method (Scheduler.hpp:533) logs the error via
   `emitErrorMessageIfAny` but **continues** — it starts pool workers, which
   begin calling `work()` on all blocks including the one in ERROR.

2. **Failed lifecycle callback at runtime** — any lifecycle callback (`pause()`,
   `resume()`, settings callbacks) that throws will transition the block to ERROR
   via the same mechanism.

3. **Explicit error transition** — any code path that calls
   `changeStateTo(ERROR)` (valid from any state per LifeCycle.hpp:75).

When `workInternal()` is called on a block in ERROR state:

- `lifecycle::State::ERROR` is not `REQUESTED_STOP` → check 1 skipped
- `lifecycle::State::ERROR` is not `STOPPED` → check 2 skipped
- The full processing pipeline executes:
  - `prepareStreams()` prepares input/output spans
  - `updateMergedInputTagAndApplySettings()` processes tags
  - `publishCachedOutputTags()` publishes tags
  - `invokeProcessBulk()` or `invokeProcessOne()` is called

If the block's `start()` callback was responsible for initializing resources
(opening files, connecting to hardware, allocating buffers), those resources are
in an indeterminate state. The processing function operates on them.

`invokeUserProvidedFunction` will catch exceptions from the processing call, and
`traverseBlockListOnce` (Scheduler.hpp:476) will propagate `work::Status::ERROR`
to the pool worker, which transitions the scheduler to ERROR. But the block's
processing function is still called at least once on bad state, and if it
happens not to throw (e.g., silently reads garbage data), it returns OK and the
block keeps being called every iteration.

### Contrast with STOPPED

A block in `STOPPED` state immediately returns `{requestedWork, 0UZ, DONE}` at
line 2028 — no processing occurs. ERROR state should receive the same early-exit
treatment. A block in ERROR is at least as broken as one that is stopped.

## Proposed Fix

Add an ERROR state check after the existing STOPPED check:

```diff
         if (this->state() == lifecycle::State::STOPPED) {
             disconnectFromUpStreamParents();
             return {requestedWork, 0UZ, DONE};
         }

+        if (this->state() == lifecycle::State::ERROR) {
+            return {requestedWork, 0UZ, DONE};
+        }
+
         // TODO: finally remove me
```

The block returns `DONE` rather than `work::Status::ERROR` because:

- The error has already been reported at the point that set the state to ERROR
  (e.g., `invokeLifecycleMethod` returns the error to the caller, and the
  scheduler's `start()` logs it via `emitErrorMessageIfAny`)
- Returning `work::Status::ERROR` would cause `traverseBlockListOnce` to
  propagate ERROR to the pool worker, which transitions the **entire scheduler**
  to ERROR — shutting down all blocks. This is disproportionate for one failed
  block, and contradicts the scheduler's existing choice to continue despite
  the failed `start()`
- `DONE` matches the STOPPED pattern and causes `traverseBlockListOnce` to
  treat the block as finished, allowing other blocks to continue

## Why This Helps

- **Safety:** Prevents calling processing functions on blocks with potentially
  uninitialized resources from a failed `start()`.
- **Correctness:** A block in ERROR state does not silently process and emit
  corrupt data.
- **Consistency:** ERROR state receives early-exit treatment matching STOPPED,
  rather than falling through to the full processing pipeline.

## Risk Assessment

**Low.** Three lines added. The check uses the same pattern as the existing
STOPPED check (line 2028). The `DONE` return value is the same as STOPPED. No
new concurrency, no new state transitions, no changes to the processing
pipeline.

Blocks that were previously in ERROR state but happened to process successfully
(returning `work::Status::OK`) will now stop processing. This is the correct
behavior — a block in ERROR should not be producing output.

## Related Observations (not part of this fix)

- The scheduler's `start()` (Scheduler.hpp:533) continues launching pool
  workers even when a block fails to start. Whether this is intentional
  "best-effort" behavior or an oversight is worth a separate review.
- A block in ERROR state could be recovered via `changeStateTo(INITIALISED)`
  (the only valid outgoing transition from ERROR). There is currently no
  mechanism in the scheduler to attempt recovery of individual blocks. With
  this fix, such a mechanism could transition the block to INITIALISED →
  RUNNING, and it would start processing again on the next `work()` call.
