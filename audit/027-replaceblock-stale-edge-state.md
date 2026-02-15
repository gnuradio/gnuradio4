# Finding 027: replaceBlock() doesn't reset edge state — replacement block runs unconnected

## Invariant
After `replaceBlock()`, edges referencing the new block must be in a state that allows reconnection.

## Code Locus
`Graph.cpp:29–56` — `replaceBlock()`.
`Scheduler.hpp:1187–1216` — `propertyCallbackReplaceBlock()`.

## Possible Violation
`replaceBlock()` rewrites edge metadata to point to the new block (lines 42–50) but:
1. Does NOT reset edge state from `Connected` to `WaitingToBeConnected`
2. Does NOT clear the stale `_sourcePort`/`_destinationPort` raw pointers (still point to old block's ports)
3. The caller (`propertyCallbackReplaceBlock`) does NOT call `connectPendingEdges()`

Result: the new block is adopted for scheduling and transitioned to RUNNING, but its ports are never connected to any buffers. The edges say `Connected` (stale) so `connectPendingEdges()` skips them. The replacement block runs indefinitely with no data flowing through it.

### Timeline
1. User sends `kReplaceBlock` message
2. `replaceBlock()` creates new block, rewrites edge metadata, returns old+new blocks
3. Old block becomes zombie (still has actual port connections)
4. New block is adopted and transitioned to RUNNING
5. New block's `work()` is called — no input/output connected → does nothing
6. Edge metadata claims `Connected` with stale port pointers → reconnection never attempted

## Fix
In `replaceBlock()` (Graph.cpp), reset edge state and clear stale port pointers for affected edges:

```cpp
if (touched) {
    edge._state           = Edge::EdgeState::WaitingToBeConnected;
    edge._sourcePort      = nullptr;
    edge._destinationPort = nullptr;
}
```

In `propertyCallbackReplaceBlock` (Scheduler.hpp), call `connectPendingEdges()` after replacement:

```cpp
auto [oldBlock, newBlockRaw] = targetGraph->replaceBlock(uniqueName, type, properties);
targetGraph->connectPendingEdges();
```

## CI Impact
All tests pass: qa_Scheduler (29 tests), qa_Graph (21 tests), qa_Block (all suites), qa_LifeCycle (8 tests).
