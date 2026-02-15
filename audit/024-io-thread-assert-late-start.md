# Finding 024: IO thread assert fires if block stopped before executor runs

## Invariant
The blocking IO thread must handle the case where the block is no longer active when the executor runs the task.

## Code Locus
`Block.hpp:2287` — `assert(lifecycle::isActive(this->state()))` inside the IO thread lambda.

## Possible Violation
`work()` for blocking IO blocks (line 2278) CAS-sets `ioThreadRunning = true` and queues the IO task on the thread pool executor (line 2286). If the block transitions to REQUESTED_STOP before the executor actually runs the queued task:

1. `work()` succeeds CAS: `ioThreadRunning = true`
2. `work()` calls `executor->execute(lambda)` — task queued but not yet running
3. External thread calls `changeStateTo(REQUESTED_STOP)` — block is now REQUESTED_STOP
4. Executor runs the lambda
5. `assert(lifecycle::isActive(this->state()))` — REQUESTED_STOP is not active → **assert failure**

In release builds: the assert is compiled out, the IO thread enters the while loop, `isActive(REQUESTED_STOP)` is false, exits immediately, transitions to STOPPED, sets `ioThreadRunning = false`. This is correct behavior but the assert is wrong.

## Fix
Replace the assert with a graceful early return:

```cpp
if (!lifecycle::isActive(this->state())) {
    ioThreadRunning.store(false);
    return;  // Block was stopped before IO thread started
}
```

This pairs with finding 023's destructor fix — the destructor waits on `ioThreadRunning`, and the IO thread sets it `false` before returning.

## Why Safe
- The early return sets `ioThreadRunning = false`, unblocking any destructor wait (finding 023)
- No work is performed on a non-active block
- The state transition to STOPPED is not needed (block is already REQUESTED_STOP or STOPPED)
- Matches the existing pattern in `workInternal()` which early-returns for non-active states

## CI Impact
All tests pass: qa_Scheduler (29 tests), qa_Block (all suites), qa_LifeCycle (8 tests).
