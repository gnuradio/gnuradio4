# Finding 016: Block::processScheduledMessages() throws in noexcept poolWorker() context

## Invariant
Any code called (directly or transitively) from `poolWorker()` must not throw. `poolWorker()` is declared `noexcept` (Scheduler.hpp line 598).

## Code Locus
`Block.hpp:1254–1260` — `Block::processScheduledMessages()`, specifically the `std::span` path.

```cpp
} else if constexpr (traits::block::can_processMessagesForPortStdSpan<Derived, TPort>) {
    self().processMessages(inPort, static_cast<std::span<const Message>>(inSpan));
    if (auto consumed = inSpan.tryConsume(inSpan.size()); !consumed) {
        throw gr::exception(std::format("Block {}::processScheduledMessages() ...", unique_name));
    }
```

## Call Chain
```
poolWorker() [noexcept]
  → std::ranges::for_each(localBlockList, &BlockModel::processScheduledMessages)  [line 639]
    → BlockWrapper<T>::processScheduledMessages() [virtual, not noexcept]  [BlockModel.hpp:851]
      → Block<T>::processScheduledMessages()  [Block.hpp:1239, not noexcept]
        → throw gr::exception(...)  [Block.hpp:1257]
          → std::terminate()
```

## Possible Violation
`tryConsume()` returns false when `isConsumeRequested()` is true — meaning the user's `processMessages()` callback already consumed the span. The `ReaderSpan` code path (line 1253) handles this gracefully with `std::ignore`. The `std::span` code path (line 1257) throws instead. This inconsistency means blocks with `std::span`-based `processMessages()` can terminate the process.

This is the same class of bug as finding 008, which fixed a throw in the **Scheduler-level** `processScheduledMessages()`. This is the **Block-level** equivalent.

## Fix
Replace the throw with `std::println` to stderr, matching the pattern from finding 008 and the silent handling at line 1253:

```cpp
if (auto consumed = inSpan.tryConsume(inSpan.size()); !consumed) {
    std::println(stderr, "Block {}::processScheduledMessages() could not consume ...", unique_name);
}
```

## Why Safe
- `tryConsume` failing means messages were already consumed — the work was done, just not by us
- Logging preserves visibility of the condition without terminating the process
- Matches the pattern at line 1253 for the `ReaderSpan` path and the fix from finding 008
- `stderr` (not `std::cerr`) matches the file's existing convention (e.g., line 1213)

## CI Impact
All existing tests pass (qa_Block: all suites, qa_Scheduler: 29 tests, qa_SchedulerMessages: 22 tests).
