# Finding 007: `stop()` Transitions Blocks While Pool Workers Still Access Them

## Area Examined

`Scheduler.hpp:stop()` (lines 718–740) — the lifecycle callback invoked when
the scheduler transitions to `REQUESTED_STOP`, and its ordering relative to
pool worker shutdown.

## Observed Behavior

`stop()` is invoked as a lifecycle callback inside `changeStateTo(REQUESTED_STOP)`.
The call sequence in `changeStateTo` is:

1. `setAndNotifyState(REQUESTED_STOP)` — scheduler state is atomically set
2. `invokeLifecycleMethod(&TDerived::stop)` — `stop()` runs synchronously

Inside `stop()`:
- Each block is transitioned to `REQUESTED_STOP`, which invokes the block's
  `stop()` callback
- Non-blocking blocks are immediately transitioned to `STOPPED`
- Sub-schedulers are stopped and their threads joined

Meanwhile, pool workers on separate threads may still be executing
`traverseBlockListOnce`, calling `work()` on those same blocks. Workers only
exit after reading the scheduler's new state (step 1), but they check state once
per iteration. A worker mid-iteration continues calling `work()` on blocks whose
`stop()` callback has already fired.

**Affected call sites:**

- `exchange()` (line 237): calls `changeStateTo(REQUESTED_STOP)` before
  `waitDone()` — workers are still running when `stop()` transitions blocks
- `SchedulerWrapper::stop()` (line 77): calls `changeStateTo(REQUESTED_STOP)` on
  sub-scheduler while the sub-scheduler's worker thread is still running
- `runAndWait()`: NOT affected — calls `waitDone()` before
  `changeStateTo(REQUESTED_STOP)`, so workers are already done

## Issue / Risk

If a block's `stop()` callback releases resources (file handles, hardware
interfaces, network connections, shared memory) that `workInternal()` accesses
during processing, there is a window for use-after-free:

1. Parent thread: `block->changeStateTo(REQUESTED_STOP)` → block's `stop()` callback
   fires, releases resource R
2. Worker thread: still inside `workInternal()`, calls `processBulk()` which
   accesses resource R → use-after-free

The window is one worker iteration (one pass through `traverseBlockListOnce`).

## Minimal Change

Add `waitDone()` at the top of `stop()`. Since the scheduler state is already
`REQUESTED_STOP` (set before the callback), workers will notice and exit. The
`waitDone()` call blocks until all workers have finished their current iteration,
ensuring no concurrent `work()` calls when block `stop()` callbacks fire.

```cpp
void stop() {
    using enum lifecycle::State;
    waitDone();  // workers exit because isActive(REQUESTED_STOP) == false
    graph::forEachBlock<TransparentBlockGroup>(*_graph, [this](auto& block) {
        // ... transition blocks (now safe — no concurrent workers)
    });
    ...
}
```

## Why This Is Safe

- In the `runAndWait()` path, workers have already exited before
  `changeStateTo(REQUESTED_STOP)`. The `waitDone()` in `stop()` sees
  `_nRunningJobs == 0` and returns immediately — no behavioral change.
- In the `exchange()` path, workers are still running. The new `waitDone()`
  waits for them to finish (they exit because `isActive(REQUESTED_STOP)` is
  false). Then blocks are safely transitioned. The existing `waitDone()` at
  line 240 becomes redundant but harmless.
- For sub-scheduler stop (`SchedulerWrapper::stop()`), the sub-scheduler's
  worker thread exits during `waitDone()`, then blocks are transitioned, then
  the thread is joined.
- `waitDone()` is a simple spin-wait on `_nRunningJobs`. It does not modify any
  state. Calling it multiple times or when no workers are running is a no-op.

## Confidence Level

**High.** The race window is directly traceable in the code. `changeStateTo`
sets state then invokes callbacks synchronously, and workers on other threads
are not synchronized before block `stop()` callbacks fire. The `runAndWait()`
path avoids this by calling `waitDone()` first, confirming the correct ordering.
