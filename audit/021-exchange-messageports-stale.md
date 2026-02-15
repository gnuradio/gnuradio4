# Finding 021: _messagePortsConnected not reset during exchange() — messages routed to old blocks

## Invariant
`_messagePortsConnected` must be `false` whenever message port connections do not match the current `_graph`.

## Code Locus
`Scheduler.hpp:152,253,330` — the flag declaration, graph swap, and flag set in `connectBlockMessagePorts()`.

## Possible Violation
`_messagePortsConnected` is set to `true` in `connectBlockMessagePorts()` (line 330) but is NEVER reset to `false`. After `exchange()` swaps `_graph` (line 253):

1. `_toChildMessagePort` still holds connections to the OLD graph's blocks
2. `_messagePortsConnected` is still `true`
3. Any message arriving via `processMessages()` (line 343) passes the `if (_messagePortsConnected)` check
4. Message is written to `_toChildMessagePort` — routed to old blocks, never reaching new blocks

### Message Loss Window
- **Active path**: Between graph swap (line 253) and `connectBlockMessagePorts()` call inside `init()` (line 271). Workers are stopped, so the window is small.
- **Non-active path**: Between graph swap and the next `init()` call. For STOPPED/ERROR/INITIALISED states without finding 020's fix, `init()` might not run until `runAndWait()` — potentially an unbounded window.

### Correct Behavior When `_messagePortsConnected` is `false`
When the flag is false, `processMessages()` (line 348) buffers messages in `_pendingMessagesToChildren`. These are forwarded to the new graph's blocks when `connectBlockMessagePorts()` re-establishes connections (lines 332–334).

## Fix
Set `_messagePortsConnected = false` before the graph swap:

```cpp
_messagePortsConnected = false; // old connections invalid for new graph; re-established by init() → connectBlockMessagePorts()
auto oldGraph = std::exchange(_graph, std::move(newGraph));
```

## Why Safe
- Messages between the flag reset and `connectBlockMessagePorts()` are safely buffered in `_pendingMessagesToChildren`
- `connectBlockMessagePorts()` drains `_pendingMessagesToChildren` and sets `_messagePortsConnected = true`
- No existing code path depends on `_messagePortsConnected` being true during `exchange()`

## CI Impact
All tests pass: qa_Scheduler (29 tests), qa_SchedulerMessages (22 tests), qa_Block (all suites), qa_LifeCycle (8 tests).
