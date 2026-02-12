# Audit Finding 002: Watchdog Uses Wrong Timeout Due to Name Shadowing

**Area:** Watchdog startup wait in `runWatchDog`
**File:** `core/include/gnuradio-4.0/Scheduler.hpp`, lines 656–692
**Severity:** Medium (watchdog can silently exit before monitoring begins)
**Risk of Fix:** Low

---

## Current Behavior

`SchedulerBase` declares two timeout members (lines 176–177):

```cpp
Annotated<gr::Size_t, "timeout", ...>          timeout_ms       = 100U;   // general no-progress sleep
Annotated<gr::Size_t, "watchdog_timeout", ...> watchdog_timeout = 1000U;  // watchdog-specific timeout
```

`runWatchDog` is called at line 543 with the watchdog-specific value:

```cpp
ioThreadPool->execute([this] { this->runWatchDog(watchdog_timeout.value, timeout_inactivity_count.value); });
```

The function signature is:

```cpp
void runWatchDog(std::size_t timeOut_ms, std::size_t timeOut_count)
```

Inside the function, the startup-wait phase (lines 662–666) computes its
deadline and polling interval using the **class member** `timeout_ms` (100ms)
instead of the **parameter** `timeOut_ms` (1000ms):

```cpp
const auto deadline      = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);       // ← class member, 100ms
const auto checkInterval = std::chrono::milliseconds(std::max(timeout_ms / 10UZ, 1UZ));                    // ← class member, 10ms
while (_valid.load(...) && _nRunningJobs->value() == 0UZ && now < deadline && lifecycle::isActive(...)) {
    std::this_thread::sleep_for(checkInterval);
}
```

The main monitoring loop at line 675 correctly uses the parameter:

```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(timeOut_ms));  // ← parameter, 1000ms
```

The names `timeout_ms` (snake_case, class member) and `timeOut_ms` (camelCase,
parameter) differ only in underscore placement. `Annotated<T>` has
`explicit(false) operator const T&()` (annotated.hpp:326), so the class member
silently converts to an integer with no compiler diagnostic.

## Issue

The watchdog's startup phase waits at most **100ms** for pool workers to begin,
instead of the intended **1000ms**. If workers are slow to start (loaded
system, large thread pool, NUMA effects), the deadline expires while
`_nRunningJobs` is still 0. The abort check at line 668 then finds
`_nRunningJobs->value() == 0UZ` and returns immediately:

```cpp
if (!_valid.load(...) || _nRunningJobs->value() == 0UZ || !lifecycle::isActive(this->state())) {
    return; // abort watchdog: scheduler inactive or jobs already finished.
}
```

**The watchdog silently exits. The graph runs unmonitored for its entire
lifetime.** No warning is logged.

Secondary effect: the startup polling interval is 10ms (100/10) instead of
100ms (1000/10) — 10× more frequent than intended, wasting CPU during the
wait.

## Proposed Fix

Replace `timeout_ms` with `timeOut_ms` on lines 662–663:

```diff
-    const auto deadline      = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
-    const auto checkInterval = std::chrono::milliseconds(std::max(timeout_ms / 10UZ, 1UZ));
+    const auto deadline      = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeOut_ms);
+    const auto checkInterval = std::chrono::milliseconds(std::max(timeOut_ms / 10UZ, 1UZ));
```

Two token changes on two lines. No behavioral change to the main monitoring
loop.

### Optional follow-up (not part of this fix)

Consider renaming the parameter to match the project's snake_case convention
(e.g., `watchdog_timeout_ms`) to eliminate the confusable name pair. This is a
style improvement and can be deferred.

## Why This Helps

- **Correctness:** The watchdog reliably waits for workers to start, using the
  timeout the caller intended.
- **Reliability:** Eliminates a silent failure mode where the graph runs with
  no deadlock/hang detection.
- **Consistency:** The startup phase and monitoring loop now both use the
  watchdog-specific timeout.

## Risk Assessment

**Low.** Two token replacements. The fix makes the startup phase use the same
parameter the monitoring loop already uses. No new code, no changed
interfaces, no altered concurrency.
