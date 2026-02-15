# Finding 020: exchange() non-active state path skips _executionOrder rebuild

## Invariant
After `exchange()`, `_executionOrder` must reference blocks from the NEW graph, not the old one.

## Code Locus
`Scheduler.hpp:264–288` — the `if (lifecycle::isActive(oldState))` guard.

## Possible Violation
When `exchange()` is called with the scheduler in a non-active, non-IDLE state (INITIALISED, STOPPED, or ERROR), the `isActive(oldState)` guard is false. The graph is swapped at line 253 but `_executionOrder` is never rebuilt:

- **INITIALISED**: `runAndWait()` → `changeStateTo(RUNNING)` triggers `start()`, which uses `_executionOrder` directly — stale blocks from old graph.
- **STOPPED/ERROR**: `runAndWait()` → `changeStateTo(INITIALISED)` triggers `reset()` (not `init()`), then `changeStateTo(RUNNING)` triggers `start()` — same stale `_executionOrder`.

IDLE is exempt because `runAndWait()`'s `IDLE→INITIALISED` transition triggers `init()` → `customInit()` which rebuilds `_executionOrder`.

This is the non-active counterpart of finding 015, which fixed the active-state path.

### Timeline
1. Scheduler constructed, `runAndWait()` completes → state is STOPPED
2. `exchange(newGraph)` called
3. `isActive(STOPPED)` is false — skip stop/wait block
4. `reset()` runs on OLD graph (before swap) — wasted work
5. Graph swapped, `isActive(STOPPED)` false — skip restart block
6. Returns — `_executionOrder` references old graph's blocks
7. `runAndWait()` → `start()` launches workers with stale `_executionOrder`

## Fix
After the `if (lifecycle::isActive(oldState))` block, add an `else` branch for non-IDLE states:

```cpp
} else if (oldState != IDLE) {
    // Non-active, non-IDLE states (INITIALISED, STOPPED, ERROR): _executionOrder was built
    // for the old graph and must be rebuilt for the new one. init() calls customInit() which
    // rebuilds _executionOrder, and connectBlockMessagePorts() for message routing.
    // IDLE is exempt because runAndWait()'s IDLE→INITIALISED transition triggers init().
    init();
}
```

## Why Safe
- `init()` calls `processScheduledMessages()`, `connectBlockMessagePorts()`, and `customInit()` — all are state-independent and safe to call in INITIALISED/STOPPED/ERROR states.
- `customInit()` reads from `_graph` (already the new graph) to rebuild `_executionOrder`.
- For STOPPED/ERROR, `runAndWait()` subsequently calls `reset()` (via STOPPED→INITIALISED) and `start()` (via INITIALISED→RUNNING). `reset()` transitions blocks and disconnects edges but does not invalidate `_executionOrder`. `start()` reconnects edges and transitions blocks to RUNNING.

## CI Impact
All tests pass: qa_Scheduler (29 tests), qa_SchedulerMessages (22 tests), qa_Block (all suites), qa_LifeCycle (8 tests).
