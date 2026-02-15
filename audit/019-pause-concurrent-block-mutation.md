# Finding 019: pause() iterates _graph->_blocks concurrently with message-driven mutations

## Invariant
`_graph->_blocks` (a `std::vector`) must not be structurally modified while being iterated.

## Code Locus
`Scheduler.hpp:767` — `pause()` callback.

```cpp
void pause() {
    graph::forEachBlock<TransparentBlockGroup>(*_graph, [this](auto& block) {
        this->emitErrorMessageIfAny("pause()", block->changeStateTo(REQUESTED_PAUSE));
```

## Possible Violation
`pause()` does NOT call `waitDone()` — pool workers remain alive (they sleep in PAUSED state). A worker's `processScheduledMessages()` (line 635) can trigger `propertyCallbackEmplaceBlock`, which calls `_graph->_blocks.emplace_back()`. If this happens while `pause()` is iterating the same vector, the `emplace_back` can reallocate the vector, invalidating the iterator.

### Timeline
1. External thread calls `scheduler.changeStateTo(REQUESTED_PAUSE)`
2. `setAndNotifyState(REQUESTED_PAUSE)` stores state
3. `pause()` callback fires, begins `forEachBlock` iteration over `_graph->_blocks`
4. Pool worker (still running) enters `processScheduledMessages`
5. "Emplace block" message triggers `propertyCallbackEmplaceBlock` → `_blocks.emplace_back()`
6. Vector reallocation invalidates the iterator in step 3

### Comparison with stop()
`stop()` calls `waitDone()` first, ensuring all pool workers have exited before iterating blocks. `pause()` has no such synchronization.

### Comparison with resume()
`resume()` (line 779) also iterates `forEachBlock` without `waitDone()`, but workers are PAUSED (sleeping), and `processScheduledMessages` runs only when `hasMessagesToProcess` is true (every `process_stream_to_message_ratio` iterations). In PAUSED state, workers still process messages (line 657), so the same race applies.

## Enforcement
Documentation-only (contract comment added to `pause()`). The correct fix would be either:
1. Snapshot `_blocks` before iteration
2. Hold `_executionOrderMutex` during iteration (but this is a different mutex than what protects `_blocks`)
3. Add a mutex protecting `_graph->_blocks` (requires architectural change)

These are beyond local invariant enforcement. The race is unlikely in practice because block mutations during pause are rare, but the invariant is structurally unprotected.

## CI Impact
No code changes beyond documentation. All tests pass.
