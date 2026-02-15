# Finding 015: exchange() does not rebuild _executionOrder after graph swap

## Invariant
`_executionOrder` must contain block pointers from the current `_graph`. After a graph swap, `customInit()` must be called to rebuild the execution order.

## Code Locus
`Scheduler.hpp` — `exchange()`, lines 230–286.

## Possible Violation
`exchange()` swaps `_graph` to a new graph (line 253), then transitions the scheduler `STOPPED → INITIALISED → RUNNING`:

1. `changeStateTo(INITIALISED)` with `oldState=STOPPED` triggers the `reset()` callback (LifeCycle.hpp line 232), NOT `init()` — because `init()` is only invoked when `oldState == IDLE` (LifeCycle.hpp line 207).
2. `changeStateTo(RUNNING)` triggers `start()`, which launches pool workers using `_executionOrder`.
3. `_executionOrder` was built by `customInit()` during the original `IDLE → INITIALISED` transition and was never updated.

Result: pool workers iterate over block pointers from the **old** graph. The new graph's blocks are transitioned to RUNNING by `start()` but are never `work()`'d. The old blocks are kept alive by shared_ptr but are disconnected.

No scheduler implements `customReset()`, so there is no hook to rebuild `_executionOrder` during the `reset()` path.

### Trigger
`propertyCallbackGraphGRC` "set grc yaml" message calls `exchange()` at line 1089.

## Fix
Call `init()` explicitly after `changeStateTo(INITIALISED)` in `exchange()`, before the transition to RUNNING. `init()` calls `connectBlockMessagePorts()` (reconnects for new graph) and `customInit()` (rebuilds `_executionOrder` from `flatten(*_graph)`).

```cpp
if (auto result = this->changeStateTo(INITIALISED); !result) {
    return std::unexpected(result.error());
}
// STOPPED → INITIALISED triggers reset() but NOT init().
// Call init() explicitly to rebuild _executionOrder and reconnect message ports.
init();
if (auto result = this->changeStateTo(RUNNING); !result) {
```

## Why Safe
- `init()` is idempotent: `customInit()` clears `_executionOrder` before rebuilding
- `connectBlockMessagePorts()` is safe to call on the fresh graph after `reset()` disconnected edges
- `reset()` already transitioned new graph's blocks to INITIALISED, providing a clean state for `init()`
- All three scheduler variants (Simple, BreadthFirst, DepthFirst) have idempotent `customInit()`

## CI Impact
All existing tests pass (qa_Scheduler: 29 tests, qa_SchedulerMessages: 22 tests, qa_Block: all suites).
