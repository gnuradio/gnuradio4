# Finding 008: `processScheduledMessages()` Throws Inside `noexcept` `poolWorker()`

## Area Examined

`Scheduler.hpp:processScheduledMessages()` (lines 348–386) — scheduler-level
message processing, specifically the unlistened error message path (lines
368–376).

## Observed Behavior

`processScheduledMessages()` contains this path:

```cpp
if (this->msgOut.buffer().streamBuffer.n_readers() == 0) {
    for (const auto& msg : messagesFromChildren) {
        if (!msg.data.has_value()) {
            throw gr::exception(...);  // <-- throws
        }
    }
    return;
}
```

This throws an exception when:
1. No external listener is subscribed to the scheduler's message output port
   (`n_readers() == 0`)
2. A child block has emitted an error message

`processScheduledMessages()` is called from `poolWorker()` at line 622.
`poolWorker()` is declared `noexcept` (line 589).

## Issue / Risk

A `throw` inside a `noexcept` function invokes `std::terminate()` — the process
is killed immediately with no cleanup, no destructor calls, no error recovery.

This triggers in a common scenario: a user creates a scheduler without
subscribing to its message output port, and any block emits an error during
processing (e.g., a failed settings change, an invalid parameter, a processing
error). The graph terminates without explanation.

The comment "convert errors to exceptions" indicates the throw was intended for
`runAndWait()` callers (where `processScheduledMessages` is also called in a
non-`noexcept` context). But the same function is called from the `noexcept`
`poolWorker`, making the throw unconditionally unsafe.

## Minimal Change

Replace the `throw` with `std::println` to stderr. This preserves the intent
(don't silently drop errors) while avoiding `std::terminate()`. The pattern is
consistent with the watchdog's error reporting (line 708).

```cpp
if (this->msgOut.buffer().streamBuffer.n_readers() == 0) {
    for (const auto& msg : messagesFromChildren) {
        if (!msg.data.has_value()) {
            std::println(std::cerr, "scheduler {}: unhandled error message: {:t}",
                this->name, msg.data.error());
        }
    }
    return;
}
```

## Why This Is Safe

- The `throw` was only reachable when no one listens on the message port. The
  fix logs to stderr instead — errors are visible, not silently dropped.
- `runAndWait()` callers who subscribe to scheduler messages are unaffected
  (they have `n_readers() > 0` and never reach this branch).
- `runAndWait()` callers who don't subscribe lose the exception, but they
  weren't handling it reliably anyway (only the first error would throw; others
  would be lost).
- No data flow or state machine changes — only the error reporting mechanism
  changes from "crash" to "log."

## Confidence Level

**High.** `poolWorker` is `noexcept`, `processScheduledMessages` throws, and
there is no try-catch between them. This is a direct path to `std::terminate()`.
