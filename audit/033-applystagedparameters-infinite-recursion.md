# Finding 033: applyStagedParameters() infinite recursion when RESET_DEFAULTS is staged

## Invariant
Recursive function calls must have a base case that terminates recursion.

## Code Locus
`Settings.hpp:859–860` — `applyStagedParameters()`.
`Settings.hpp:600–608` — `resetDefaults()`.

## Possible Violation
```cpp
// applyStagedParameters() line 859:
if (_stagedParameters.contains(gr::tag::RESET_DEFAULTS)) {
    resetDefaults();   // ← calls applyStagedParameters() internally
}
```

```cpp
// resetDefaults() line 600-608:
void resetDefaults() override {
    auto ctx = SettingsCtx{...};
    addStoredParameters(_defaultParameters, ctx);
    std::ignore = activateContext();
    std::ignore = applyStagedParameters();   // ← recursive call
    // ...
}
```

When `RESET_DEFAULTS` is in `_stagedParameters`:
1. Outer `applyStagedParameters()` detects `RESET_DEFAULTS` (line 859)
2. Calls `resetDefaults()` (line 860)
3. `resetDefaults()` calls `activateContext()` — inserts default parameters into `_stagedParameters` but does NOT remove `RESET_DEFAULTS`
4. `resetDefaults()` calls `applyStagedParameters()` — inner recursive call
5. Inner `applyStagedParameters()` detects `RESET_DEFAULTS` (still present at line 859) → calls `resetDefaults()` again
6. Infinite recursion → stack overflow → crash

`_stagedParameters` is only cleared at line 944 (`_stagedParameters.clear()`), which is at the END of `applyStagedParameters()`. The `RESET_DEFAULTS` key is never erased before the recursive call, so each recursive call sees it and recurses again.

### Trigger path
1. User stages `RESET_DEFAULTS` along with other parameters (e.g., via a tag containing `{reset_default: true, gain: 5}`)
2. Scheduler calls `applyChangedSettings()` → `applyStagedParameters()`
3. Infinite recursion begins at step described above

### Current status
Latent — `RESET_DEFAULTS` is defined in `Tag.hpp:182` but never sent in current test/production code. The test suite explicitly excludes it from default tag tests (`qa_Tags.cpp:118`). However, the tag IS part of the public API and can be sent by any user block or external message.

## Fix
Erase `RESET_DEFAULTS` from `_stagedParameters` before calling `resetDefaults()`:
```cpp
if (_stagedParameters.contains(gr::tag::RESET_DEFAULTS)) {
    _stagedParameters.erase(gr::tag::RESET_DEFAULTS);
    resetDefaults();
}
```

## CI Impact
All tests pass: qa_Settings, qa_Scheduler, qa_Graph, qa_Block, qa_LifeCycle.
