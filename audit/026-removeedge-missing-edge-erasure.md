# Finding 026: removeEdgeBySourcePort() doesn't remove edge from _edges — zombie edge resurfaces

## Invariant
Graph mutation operations must keep port connections and edge metadata in sync.

## Code Locus
`Graph.hpp:520–531` — `removeEdgeBySourcePort()`.

## Possible Violation
`removeEdgeBySourcePort()` disconnects the port (line 528) but does NOT remove the corresponding edge from `_edges`. Compare with the symmetric operation `emplaceEdge()` (line 490–518), which both connects the port AND adds an edge to `_edges`.

After `removeEdgeBySourcePort()`:
1. Port is disconnected — data no longer flows
2. Edge metadata remains in `_edges` with state still `Connected`
3. On scheduler restart, `disconnectAllEdges()` resets all edge states to `WaitingToBeConnected`
4. `connectPendingEdges()` reconnects the "removed" edge — it resurfaces as a zombie

### Trigger path
1. User sends `kRemoveEdge` message → `propertyCallbackRemoveEdge` (Scheduler.hpp:911)
2. Calls `targetGraph->removeEdgeBySourcePort(sourceBlock, sourcePort)`
3. Port disconnected, edge metadata retained
4. Scheduler restarts (or `reconnectAllEdges()` is called)
5. Zombie edge is reconnected

## Fix
After disconnecting the port, also remove matching edge metadata from `_edges`:

```cpp
const PortDefinition sourcePortDef{std::string(sourcePort)};
_edges.erase(std::remove_if(_edges.begin(), _edges.end(),
                 [&](const Edge& edge) {
                     return edge.sourceBlock() == *sourceBlockIt
                         && edge.sourcePortDefinition().definition == sourcePortDef.definition;
                 }),
    _edges.end());
```

## CI Impact
All tests pass: qa_Scheduler (29 tests), qa_Graph (21 tests), qa_Block (all suites), qa_LifeCycle (8 tests).
