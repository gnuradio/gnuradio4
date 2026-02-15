# Finding 017: User processMessages() callback can throw into noexcept poolWorker()

## Invariant
All user-provided callbacks invoked from the `poolWorker()` call chain must be exception-safe. `poolWorker()` is declared `noexcept` (Scheduler.hpp line 602).

## Code Locus
`Block.hpp:1251` and `Block.hpp:1255` — `Block::processScheduledMessages()`.

```cpp
self().processMessages(inPort, inSpan);           // ReaderSpan path
self().processMessages(inPort, static_cast<...>); // std::span path
```

## Call Chain
```
poolWorker() [noexcept]
  → for_each(localBlockList, &BlockModel::processScheduledMessages) [line 643]
    → Block::processScheduledMessages() [NOT noexcept]
      → self().processMessages(inPort, ...) [user code, CAN throw]
        → std::terminate()
```

## Possible Violation
User `processBulk()` and `processOne()` are wrapped in `invokeUserProvidedFunction()` (Block.hpp line 800), which catches all exceptions and emits error messages via `emitErrorMessageIfAny`. The user's `processMessages()` callback was NOT wrapped — a throw from user code would propagate uncaught into the noexcept `poolWorker()`, invoking `std::terminate()`.

This completes the exception containment boundary that finding 008 (Scheduler-level) and finding 016 (tryConsume) partially addressed. All three throw paths exist in the same call chain from `poolWorker()`.

## Fix
Wrap user `processMessages` calls in `invokeUserProvidedFunction`, the established pattern for user callback exception safety:

```cpp
invokeUserProvidedFunction("processMessages(ReaderSpan)", [&] { self().processMessages(inPort, inSpan); });
invokeUserProvidedFunction("processMessages(std::span)", [&] { self().processMessages(inPort, ...); });
```

## Why Safe
- `invokeUserProvidedFunction` is `noexcept`, catches all exceptions, and emits error messages
- Matches the pattern already used for `processBulk`/`processOne` (lines 2104, 2146, 2148)
- If user's `processMessages` IS noexcept, `invokeUserProvidedFunction` takes the fast path (no try-catch)
- The lambda captures by reference, matching the existing call semantics exactly

## CI Impact
All tests pass: qa_Scheduler (29 tests), qa_SchedulerMessages (22 tests), qa_Block (all suites), qa_LifeCycle (8 tests).
