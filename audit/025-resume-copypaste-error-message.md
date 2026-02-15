# Finding 025: resume() error message says "init()" — copy-paste bug

## Invariant
Error messages must identify the correct call site for debugging.

## Code Locus
`Scheduler.hpp:799` — inside the `resume()` callback.

## Possible Violation
```cpp
void resume() {
    auto result = connectPendingEdges();
    if (!result) {
        this->emitErrorMessage("init()", "Failed to connect blocks in graph");  // ← says "init()"
    }
```

The error message identifies the call site as `"init()"` but the actual call site is `resume()`. This is a copy-paste from the `start()` function (which also calls `connectPendingEdges()`). If `connectPendingEdges()` fails during resume, the error log misleadingly points to `init()`, complicating debugging.

## Fix
Change `"init()"` to `"resume()"`:

```cpp
this->emitErrorMessage("resume()", "Failed to connect blocks in graph");
```

## CI Impact
All tests pass. No behavioral change.
