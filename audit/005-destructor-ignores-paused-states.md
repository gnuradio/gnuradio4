# Audit Finding 005: Scheduler Destructor Hangs When State Is PAUSED or REQUESTED_PAUSE

**Area:** `SchedulerBase` destructor lifecycle teardown
**File:** `core/include/gnuradio-4.0/Scheduler.hpp`, lines 206–223
**Severity:** High (destructor hangs indefinitely or calls `std::abort`)
**Risk of Fix:** Low

---

## Current Behavior

The `~SchedulerBase()` destructor at line 206 initiates shutdown only when the
scheduler is in RUNNING state:

```cpp
~SchedulerBase() {
    if (this->state() == lifecycle::RUNNING) {               // ← only checks RUNNING
        if (auto e = this->changeStateTo(lifecycle::REQUESTED_STOP); !e) {
            std::println(std::cerr, "Failed to stop execution at destruction of scheduler: ...");
            std::abort();
        }
    }
    waitDone();                                               // ← polls _nRunningJobs until 0
    _valid.store(false, std::memory_order_release);
    while (_nWatchdogsRunning.load() != 0) { /* spin */ }
    _executionOrder.reset();
}
```

`waitDone()` (line 422) spins until all pool workers exit:

```cpp
void waitDone() {
    while (_nRunningJobs->value() > 0UZ) {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    }
}
```

## Issue

`lifecycle::isActive()` returns true for three states: **RUNNING**,
**REQUESTED_PAUSE**, and **PAUSED** (LifeCycle.hpp:70). The destructor only
checks RUNNING, leaving two active states unhandled:

### Case 1: State is PAUSED

- The `if` condition is false — no stop is initiated
- Pool workers are in the PAUSED branch of the main loop, sleeping for
  `timeout_ms` then re-checking state. They never exit because the state is
  still PAUSED (`isActive(PAUSED)` is true)
- `_nRunningJobs` never reaches 0
- **`waitDone()` spins forever. The destructor hangs.**

### Case 2: State is REQUESTED_PAUSE

- The `if` condition is false — no stop is initiated
- Pool workers are in the `else` branch, sleeping and re-checking
- Same hang as Case 1: `waitDone()` never returns

### Case 2b: If the destructor DID try to stop from REQUESTED_PAUSE

Even if the check were broadened to `isActive()`, a direct
`changeStateTo(REQUESTED_STOP)` from REQUESTED_PAUSE is **invalid** per the
state machine (LifeCycle.hpp:83 — REQUESTED_PAUSE can only transition to
PAUSED). The `changeStateTo` would return an error, triggering the
`std::abort()` path.

## Proposed Fix

Replace the RUNNING-only check with `isActive()`, and handle the
REQUESTED_PAUSE intermediate state by completing the pause first:

```diff
     ~SchedulerBase() {
-        if (this->state() == lifecycle::RUNNING) {
-            if (auto e = this->changeStateTo(lifecycle::REQUESTED_STOP); !e) {
-                std::println(std::cerr, "Failed to stop execution at destruction of scheduler: {} ({})", e.error().message, e.error().srcLoc());
-                std::abort();
-            }
+        if (this->state() == lifecycle::REQUESTED_PAUSE) {
+            // REQUESTED_PAUSE can only transition to PAUSED; complete the pause first
+            std::ignore = this->changeStateTo(lifecycle::PAUSED);
+        }
+        if (lifecycle::isActive(this->state())) {
+            if (auto e = this->changeStateTo(lifecycle::REQUESTED_STOP); !e) {
+                std::println(std::cerr, "Failed to stop execution at destruction of scheduler: {} ({})", e.error().message, e.error().srcLoc());
+                std::abort();
+            }
         }
         waitDone();
```

State machine paths covered:
- **RUNNING** → REQUESTED_STOP (valid, line 82) — existing behavior preserved
- **PAUSED** → REQUESTED_STOP (valid, line 84) — now handled
- **REQUESTED_PAUSE** → PAUSED → REQUESTED_STOP (valid, lines 83 → 84) —
  now handled

Non-active states (IDLE, INITIALISED, STOPPED, ERROR) skip the block, as
before.

## Why This Helps

- **Correctness:** The destructor terminates cleanly regardless of which active
  state the scheduler is in.
- **Safety:** Eliminates a hang (infinite loop in `waitDone`) that would
  manifest as a frozen process on shutdown.
- **Robustness:** The REQUESTED_PAUSE path is rare in practice (destructor
  called during a narrow transition window), but destructors must handle all
  reachable states.

## Risk Assessment

**Low.** The fix adds two conditional checks using existing state machine
transitions. Both `changeStateTo(PAUSED)` from REQUESTED_PAUSE and
`changeStateTo(REQUESTED_STOP)` from PAUSED are well-defined transitions
already exercised through `pause()` and `stop()`. No new concurrency, no new
code paths in the worker threads.

Pool workers in PAUSED state will notice the REQUESTED_STOP transition within
one sleep cycle (~100ms default), then exit their main loop. This brief delay
in the destructor is acceptable since the alternative is hanging forever.
