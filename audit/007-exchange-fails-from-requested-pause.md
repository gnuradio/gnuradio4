# Audit Finding 007: `exchange()` Fails When Scheduler State Is REQUESTED_PAUSE

**Area:** Graph exchange stop-down sequence
**File:** `core/include/gnuradio-4.0/Scheduler.hpp`, lines 225–237
**Severity:** Medium (graph exchange silently fails, returns error to caller)
**Risk of Fix:** Low

---

## Current Behavior

`exchange()` stops the scheduler before swapping the graph:

```cpp
std::expected<meta::indirect<Graph>, Error> exchange(meta::indirect<Graph>&& newGraph, ...) {
    using enum lifecycle::State;
    const auto oldState = this->state();
    if (lifecycle::isActive(oldState)) {                          // true for RUNNING, REQUESTED_PAUSE, PAUSED
        if (auto result = this->changeStateTo(REQUESTED_STOP); !result) {  // ← line 229
            return std::unexpected(result.error());               // ← fails for REQUESTED_PAUSE
        }
        waitDone();
        if (auto result = this->changeStateTo(STOPPED); !result) {
            return std::unexpected(result.error());
        }
    }
    // ... swap graph, restore state ...
}
```

The restore logic at lines 253–273 correctly handles the REQUESTED_PAUSE state
via multi-step transitions (RUNNING → REQUESTED_PAUSE):

```cpp
if (oldState == REQUESTED_PAUSE) {
    if (auto result = this->changeStateTo(REQUESTED_PAUSE); !result) { ... }
} else if (oldState == PAUSED) {
    if (auto result = this->changeStateTo(REQUESTED_PAUSE); !result) { ... }
    if (auto result = this->changeStateTo(PAUSED); !result) { ... }
}
```

The author was aware of the state machine constraints for restoration but missed
the same constraint in the stop-down path.

## Issue

Per the state machine (`LifeCycle.hpp:83`), REQUESTED_PAUSE can only transition
to PAUSED. The direct transition REQUESTED_PAUSE → REQUESTED_STOP is invalid.

When `exchange()` is called while the scheduler is in REQUESTED_PAUSE:

1. `isActive(REQUESTED_PAUSE)` returns true (line 228)
2. `changeStateTo(REQUESTED_STOP)` returns an error (invalid transition)
3. `exchange()` returns `std::unexpected(result.error())`
4. The graph exchange does not occur
5. The caller receives an error

This affects two call sites:

- **`propertyCallbackGraphGRC`** (line 1048): The message handler for "set GRC
  YAML" calls `makeAllZombies()` and then `exchange()`. If `exchange()` fails,
  all blocks have already been zombified but the new graph is not installed. The
  scheduler is left with an empty graph and zombified blocks.

- **`SchedulerWrapper::setGraph()`** (`SchedulerModel.hpp:53`): Discards the
  error with `std::ignore`, so the graph replacement silently fails.

The REQUESTED_PAUSE state is a narrow window (between the scheduler receiving
a pause request and completing the transition), but `propertyCallbackGraphGRC`
is a message handler that runs inside `processScheduledMessages()` — the same
context where the pause transition is being processed.

## Proposed Fix

Complete the pause before requesting stop, matching the pattern from Finding
005:

```diff
     const auto oldState = this->state();
     if (lifecycle::isActive(oldState)) { // need to stop running scheduler
+        if (this->state() == REQUESTED_PAUSE) {
+            if (auto result = this->changeStateTo(PAUSED); !result) {
+                return std::unexpected(result.error());
+            }
+        }
         if (auto result = this->changeStateTo(REQUESTED_STOP); !result) {
             return std::unexpected(result.error());
         }
```

State machine paths after the fix:

- **RUNNING** → REQUESTED_STOP (valid, line 82) — existing behavior preserved
- **PAUSED** → REQUESTED_STOP (valid, line 84) — existing behavior preserved
- **REQUESTED_PAUSE** → PAUSED → REQUESTED_STOP (valid, lines 83 → 84) — now
  handled

## Why This Helps

- **Correctness:** `exchange()` succeeds regardless of which active state the
  scheduler is in.
- **Safety:** Prevents `propertyCallbackGraphGRC` from leaving the scheduler in
  a broken state (blocks zombified but no new graph installed).
- **Consistency:** The stop-down path and the restore path now both correctly
  handle the REQUESTED_PAUSE intermediate state.

## Risk Assessment

**Low.** Three lines added, using the same two-step pattern already used in the
restore section of the same function (lines 267–272) and proposed for the
destructor (Finding 005). Both `changeStateTo(PAUSED)` from REQUESTED_PAUSE and
`changeStateTo(REQUESTED_STOP)` from PAUSED are well-defined, well-exercised
transitions.
