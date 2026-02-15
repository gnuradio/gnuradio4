# Finding 006: `runAndWait()` Returns Success and Abandons Blocks on Scheduler ERROR

## Area Examined

`Scheduler.hpp:runAndWait()` (lines 388–428) — scheduler completion path after
workers exit, and its interaction with block lifecycle cleanup.

## Observed Behavior

When a block's `work()` returns `work::Status::ERROR` during execution:

1. `poolWorker` (line 630) transitions the **scheduler** to `ERROR` and breaks
2. Other poolWorkers see `isActive(ERROR) == false` and exit the while loop
3. All workers exit → `_nRunningJobs` reaches 0 → `waitDone()` returns
4. `runAndWait()` line 415: `this->state() == RUNNING` is false (it's ERROR) → skip
5. `runAndWait()` line 421: `this->state() == REQUESTED_STOP` is false → skip
6. `runAndWait()` returns `{}` — **success**

No block transitions occur. The scheduler's `stop()` method (which calls
`changeStateTo(REQUESTED_STOP)` on every block) is only invoked during the
`RUNNING → REQUESTED_STOP` transition. That transition is skipped when the
scheduler is in ERROR.

## Issue / Risk

Two concrete problems:

1. **Silent success on failure.** `runAndWait()` returns success when the
   scheduler is in ERROR state. The caller cannot distinguish a clean completion
   from a failed execution.

2. **Blocks abandoned in RUNNING state.** All blocks remain in lifecycle state
   RUNNING with no one calling `work()` on them. Their `stop()` callbacks never
   fire. Resources held by those callbacks (file handles, network connections,
   hardware interfaces) are not released until block destruction — at which point
   scheduler services (message routing) may already be gone.

   This also breaks the recovery path: if the caller later calls `runAndWait()`
   again, line 392 transitions the scheduler from ERROR → INITIALISED, which
   invokes `reset()`. `reset()` then calls `block->changeStateTo(INITIALISED)`
   on blocks still in RUNNING state — but `RUNNING → INITIALISED` is not a valid
   transition. Recovery silently fails.

## Minimal Change

After `waitDone()`, insert an ERROR branch before the existing RUNNING cleanup.
It mirrors the `stop()` method pattern: transition all active blocks through
`REQUESTED_STOP → STOPPED`, then return an error to the caller.

```cpp
if (this->state() == ERROR) {
    graph::forEachBlock<TransparentBlockGroup>(*_graph, [this](auto& block) {
        if (lifecycle::isActive(block->state())) {
            this->emitErrorMessageIfAny("runAndWait() ERROR cleanup -> REQUESTED_STOP",
                block->changeStateTo(REQUESTED_STOP));
            if (!block->isBlocking()) {
                this->emitErrorMessageIfAny("runAndWait() ERROR cleanup -> STOPPED",
                    block->changeStateTo(STOPPED));
            }
        }
    });
    processScheduledMessages();
    return std::unexpected(Error("runAndWait(): scheduler entered ERROR state during execution"));
}
```

## Why This Is Safe

- Uses the same `REQUESTED_STOP → STOPPED` transition sequence as `stop()`.
  Both transitions are valid from RUNNING (the state blocks are in).
- Block `stop()` callbacks fire via `changeStateTo(REQUESTED_STOP)`, ensuring
  resource cleanup happens through the normal path.
- Non-blocking blocks are immediately moved to STOPPED (same as `stop()`).
  Blocking blocks remain in REQUESTED_STOP and are cleaned up by their own
  IO thread or destructor.
- The existing RUNNING and REQUESTED_STOP branches remain unchanged and are
  unreachable when the scheduler is in ERROR, so no existing behavior is altered.
- Returning `std::unexpected` gives callers visibility into the failure.

## Confidence Level

**High.** The code path is directly traceable: `runAndWait()` has no branch
for `this->state() == ERROR` after `waitDone()`, and the state machine
confirms that `isActive(ERROR)` returns false, causing both the RUNNING and
REQUESTED_STOP branches to be skipped.
