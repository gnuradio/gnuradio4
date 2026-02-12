# GNU Radio 4 Core Runtime Audit Report

**Scope:** Core scheduler and block lifecycle interaction
**Files:** `Scheduler.hpp`, `Block.hpp`, `LifeCycle.hpp`
**Date:** 2026-02-12
**Status:** 8 findings identified, all fixes applied

---

## Executive Summary

This audit examined the core runtime of GNU Radio 4 beta, focusing on the
scheduler/block lifecycle interaction, the `poolWorker` execution loop, dynamic
graph modification paths, and destructor safety. Eight issues were identified
across two files (`Scheduler.hpp` and `Block.hpp`), ranging from silent
behavioral bugs to potential hangs and data corruption. All fixes are minimal
and isolated: **+47 lines, -7 lines** across two files.

No large rewrites or architectural changes are proposed. Every fix is small
enough for a single short PR and uses patterns already present in the codebase.

### Findings at a Glance

| # | Severity | Area | Issue | Fix Size |
|---|----------|------|-------|----------|
| [001](#finding-001) | Low-Med | `poolWorker` loop | Stale `activeState` delays stop/pause by up to 15 iterations | 1 line moved |
| [002](#finding-002) | Medium | `runWatchDog` | Name shadowing causes watchdog to use 100ms instead of 1000ms timeout | 2 tokens changed |
| [003](#finding-003) | Medium | `workInternal` DONE path | `stop()` lifecycle callback skipped when processing returns DONE | 1 line added |
| [004](#finding-004) | High | `replaceBlock` handler | Replacement block never added to scheduler — silently dead | ~20 lines added |
| [005](#finding-005) | High | `~SchedulerBase` | Destructor hangs forever if scheduler is PAUSED or REQUESTED_PAUSE | 3 lines added |
| [006](#finding-006) | Medium | `workInternal` ERROR path | Block in ERROR state proceeds through full processing pipeline | 3 lines added |
| [007](#finding-007) | Medium | `exchange()` | Graph exchange fails from REQUESTED_PAUSE (invalid transition) | 3 lines added |
| [008](#finding-008) | Medium | `poolWorker` messages | Scheduler messages orphaned when runner 0 exits before other runners | 1 line removed |

### Root Cause Patterns

Two systemic patterns account for five of the eight findings:

1. **REQUESTED_PAUSE as a trap state** (Findings 005, 007): The state machine
   only allows REQUESTED_PAUSE to transition to PAUSED. Code that uses
   `isActive()` to gate stop-down logic (correctly catching RUNNING,
   REQUESTED_PAUSE, and PAUSED) then calls `changeStateTo(REQUESTED_STOP)`
   directly, which fails for REQUESTED_PAUSE. The fix is always the same:
   complete the pause first (REQUESTED_PAUSE -> PAUSED), then stop.

2. **Missing state checks in `workInternal`** (Findings 003, 006): The central
   `workInternal()` method checks for REQUESTED_STOP and STOPPED but not for
   ERROR or the DONE-return lifecycle path. Both omissions allow blocks to
   either process data in broken states or skip required lifecycle callbacks.

---

## Findings

### Finding 001

**poolWorker Stale `activeState` Cache**
Severity: Low-Medium | File: `Scheduler.hpp` | Risk of Fix: Low

The `poolWorker` loop caches the scheduler's lifecycle state in a local variable
`activeState`, but only refreshes it inside the `if (hasMessagesToProcess)`
branch — which executes once every `process_stream_to_message_ratio` iterations
(default: 16). On the other 15 iterations, the loop uses a stale value.

When an external thread requests stop or pause, pool workers continue calling
`traverseBlockListOnce` for up to 15 additional iterations before noticing. This
couples message-processing frequency to state-transition latency — an unintended
side effect that makes `process_stream_to_message_ratio` behave differently from
its documentation.

**Fix:** Move `activeState = this->state()` from inside the conditional to after
it, so the state is refreshed unconditionally every iteration. One atomic load
(`mov` on x86) per iteration — unmeasurable overhead.

```diff
             std::ranges::for_each(localBlockList, &BlockModel::processScheduledMessages);
-            activeState = this->state();
             msgToCount++;
         } else {
             // ...
         }

+        activeState = this->state();
+
         if (activeState == RUNNING) {
```

---

### Finding 002

**Watchdog Uses Wrong Timeout Due to Name Shadowing**
Severity: Medium | File: `Scheduler.hpp` | Risk of Fix: Low

`runWatchDog(std::size_t timeOut_ms, ...)` receives the watchdog timeout
(1000ms) as a parameter, but its startup-wait phase uses the class member
`timeout_ms` (100ms) instead. The names differ only in underscore placement
(`timeout_ms` vs `timeOut_ms`), and `Annotated<T>`'s `explicit(false)` implicit
conversion silences any compiler diagnostic.

The watchdog waits at most 100ms for pool workers to start. On a loaded system,
workers may not have started yet, and the watchdog exits silently — the graph
runs unmonitored for its entire lifetime.

**Fix:** Replace `timeout_ms` with `timeOut_ms` on the two affected lines.

```diff
-    const auto deadline      = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
-    const auto checkInterval = std::chrono::milliseconds(std::max(timeout_ms / 10UZ, 1UZ));
+    const auto deadline      = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeOut_ms);
+    const auto checkInterval = std::chrono::milliseconds(std::max(timeOut_ms / 10UZ, 1UZ));
```

---

### Finding 003

**User-Returned DONE Bypasses `stop()` Lifecycle Callback**
Severity: Medium | File: `Block.hpp` | Risk of Fix: Low

When `processBulk`/`processOne` returns `work::Status::DONE`, the block
transitions directly RUNNING -> STOPPED via `setAndNotifyState()`, bypassing
`changeStateTo(REQUESTED_STOP)`. This skips the `stop()` lifecycle callback.

The EOS path (line 2068) and `requestStop()` both correctly go through
`changeStateTo(REQUESTED_STOP)`, which invokes `stop()`. Only the user-DONE
path skips it. Any block relying on `stop()` for cleanup (flushing buffers,
closing files, releasing hardware) silently misses that cleanup.

**Fix:** Add `changeStateTo(REQUESTED_STOP)` before the direct state set,
matching the existing EOS path pattern.

```diff
         if (userReturnStatus == DONE) {
+            emitErrorMessageIfAny("workInternal() DONE", this->changeStateTo(lifecycle::State::REQUESTED_STOP));
             this->setAndNotifyState(lifecycle::State::STOPPED);
             publishEoS(outputSpans);
         }
```

---

### Finding 004

**Replaced Block Never Adopted by Scheduler Workers**
Severity: High | File: `Scheduler.hpp` | Risk of Fix: Low

`propertyCallbackReplaceBlock` creates a new block via `Graph::replaceBlock()`
and zombifies the old block, but never adds the new block to `_adoptionBlocks`.
Compare with `propertyCallbackEmplaceBlock`, which has full adoption logic
(add to `_adoptionBlocks`, transition IDLE -> INITIALISED -> RUNNING).

The replacement block exists in the graph topology with correct edge references
but is never scheduled. No pool worker calls `work()` on it. Data flows up to
its input ports and stops. The caller receives a success notification.

**Fix:** Add the same adoption logic from `propertyCallbackEmplaceBlock` —
push to `_adoptionBlocks` and transition to RUNNING.

```diff
     auto [oldBlock, newBlockRaw] = targetGraph->replaceBlock(uniqueName, type, properties);
     makeZombie(std::move(oldBlock));

+    if (lifecycle::isActive(this->state())) {
+        const auto nBatches = _adoptionBlocks.size();
+        if (nBatches > 0) {
+            std::lock_guard guard(_adoptionBlocksMutex);
+            auto            blockAddress = reinterpret_cast<std::uintptr_t>(&newBlockRaw);
+            auto            runnerIndex  = (blockAddress / sizeof(void*)) % nBatches;
+            _adoptionBlocks[runnerIndex].push_back(newBlockRaw);
+
+            switch (newBlockRaw->state()) {
+            case STOPPED:
+            case IDLE:
+                this->emitErrorMessageIfAny("propertyCallbackReplaceBlock -> INITIALISED", newBlockRaw->changeStateTo(INITIALISED));
+                this->emitErrorMessageIfAny("propertyCallbackReplaceBlock -> RUNNING", newBlockRaw->changeStateTo(RUNNING));
+                break;
+            case INITIALISED:
+                this->emitErrorMessageIfAny("propertyCallbackReplaceBlock -> RUNNING", newBlockRaw->changeStateTo(RUNNING));
+                break;
+            case RUNNING:
+            case REQUESTED_PAUSE:
+            case PAUSED:
+            case REQUESTED_STOP:
+            case ERROR:
+                this->emitErrorMessage("propertyCallbackReplaceBlock",
+                    std::format("Unexpected block state during replacement: {}", magic_enum::enum_name(newBlockRaw->state())));
+                break;
+            }
+        }
+    }
+
     std::optional<Message> result = gr::Message{};
```

---

### Finding 005

**Scheduler Destructor Hangs When PAUSED or REQUESTED_PAUSE**
Severity: High | File: `Scheduler.hpp` | Risk of Fix: Low

`~SchedulerBase()` only checks `this->state() == lifecycle::RUNNING` before
initiating stop. If the scheduler is PAUSED or REQUESTED_PAUSE, no stop is
initiated, and `waitDone()` spins forever — pool workers never exit because
`isActive(PAUSED)` remains true.

For REQUESTED_PAUSE, even a broadened `isActive()` check would fail:
REQUESTED_PAUSE -> REQUESTED_STOP is invalid per the state machine. The
destructor must complete the pause first.

**Fix:** Complete REQUESTED_PAUSE -> PAUSED, then use `isActive()` to gate the
stop.

```diff
     ~SchedulerBase() {
-        if (this->state() == lifecycle::RUNNING) {
+        if (this->state() == lifecycle::REQUESTED_PAUSE) {
+            std::ignore = this->changeStateTo(lifecycle::PAUSED);
+        }
+        if (lifecycle::isActive(this->state())) {
             if (auto e = this->changeStateTo(lifecycle::REQUESTED_STOP); !e) {
```

---

### Finding 006

**`workInternal` Processes Data on Block in ERROR State**
Severity: Medium | File: `Block.hpp` | Risk of Fix: Low

`workInternal()` checks for REQUESTED_STOP and STOPPED but not ERROR. A block
whose `start()` callback threw an exception is set to ERROR state by
`invokeLifecycleMethod`, but the scheduler's `start()` logs the error and
continues. Pool workers then call `work()` on the ERROR block, which falls
through to `processBulk`/`processOne` — operating on potentially uninitialized
resources.

If the processing function happens not to throw, it returns OK and the block
keeps being called every iteration, possibly emitting corrupt data.

**Fix:** Add an ERROR state check after the existing STOPPED check, returning
DONE to cleanly stop the block.

```diff
         if (this->state() == lifecycle::State::STOPPED) {
             disconnectFromUpStreamParents();
             return {requestedWork, 0UZ, DONE};
         }

+        if (this->state() == lifecycle::State::ERROR) {
+            return {requestedWork, 0UZ, DONE};
+        }
```

---

### Finding 007

**`exchange()` Fails When Scheduler State Is REQUESTED_PAUSE**
Severity: Medium | File: `Scheduler.hpp` | Risk of Fix: Low

`exchange()` uses `isActive()` to gate its stop-down sequence (correctly
catching RUNNING, REQUESTED_PAUSE, PAUSED), but then calls
`changeStateTo(REQUESTED_STOP)` directly — which fails for REQUESTED_PAUSE.
The graph exchange returns an error and the swap never occurs.

The restore logic in the same function (lines 262-273) correctly handles
REQUESTED_PAUSE via multi-step transitions. The stop-down path was missed.

In `propertyCallbackGraphGRC`, `makeAllZombies()` is called before `exchange()`.
If `exchange()` fails, all blocks are zombified but no new graph is installed.

**Fix:** Complete the pause before requesting stop, matching Finding 005.

```diff
         if (lifecycle::isActive(oldState)) { // need to stop running scheduler
+            if (this->state() == REQUESTED_PAUSE) {
+                if (auto result = this->changeStateTo(PAUSED); !result) {
+                    return std::unexpected(result.error());
+                }
+            }
             if (auto result = this->changeStateTo(REQUESTED_STOP); !result) {
```

---

### Finding 008

**Scheduler Messages Orphaned When Runner 0 Exits Early**
Severity: Medium | File: `Scheduler.hpp` | Risk of Fix: Low

In the `poolWorker` loop, scheduler-level messages are only processed when
`runnerID == 0UZ`. The fallback `nRunningJobs->value() == 0UZ` is dead code
(can never be true inside a running pool worker — the caller contributes 1).

When runner 0's blocks all finish and it exits the loop, no remaining runner
processes scheduler messages. Dynamic graph modifications (`kEmplaceBlock`,
`kRemoveBlock`, etc.) accumulate in the ring buffer unprocessed.

The `processScheduledMessages()` method already has an internal atomic flag
(`_processingScheduledMessages`) that prevents concurrent execution — the
runner-0 guard was redundant.

**Fix:** Remove the guard so any runner can process scheduler messages.

```diff
-                if (runnerID == 0UZ || nRunningJobs->value() == 0UZ) {
-                    this->processScheduledMessages();
-                }
+                this->processScheduledMessages();
```

---

## Files Changed

| File | Lines Added | Lines Removed | Net |
|------|------------|---------------|-----|
| `core/include/gnuradio-4.0/Scheduler.hpp` | 42 | 7 | +35 |
| `core/include/gnuradio-4.0/Block.hpp` | 5 | 0 | +5 |
| **Total** | **47** | **7** | **+40** |

## Observations for Future Work

The following items were noted during the audit but are outside the scope of
these fixes. They are recorded here for potential future investigation:

1. **`changeStateTo` uses load-then-store, not CAS.** The state machine in
   `LifeCycle.hpp` reads the current state, validates the transition, then
   writes the new state — without a compare-and-swap. Under concurrent
   `changeStateTo` calls from different threads, a TOCTOU race can cause
   invalid transitions. This is an architectural concern that would require
   a broader design discussion.

2. **BFS/DFS schedulers silently drop unreachable blocks.** The `BreadthFirst`
   and `DepthFirst` schedulers only schedule blocks reachable from source blocks
   (blocks with outgoing but no incoming edges). Isolated blocks or blocks in
   closed cycles without an external source are silently excluded from
   execution. The `Simple` scheduler includes all blocks. No warning is emitted
   when blocks are dropped.

3. **`cleanupZombieBlocks` calls `changeStateTo` under mutex.** Lifecycle
   callbacks (potentially user code) execute while `_zombieBlocksMutex` is held.
   For blocks in STOPPED/IDLE/ERROR state (the delete path), the callbacks are
   no-ops. For blocks in transitional states, the callbacks may interact with
   shared state.

4. **`makeAllZombies` attempts invalid REQUESTED_PAUSE -> REQUESTED_STOP.**
   The same root cause as Findings 005/007 appears in `makeAllZombies()`, which
   handles REQUESTED_PAUSE blocks by directly calling
   `changeStateTo(REQUESTED_STOP)`. The transition fails and the error is
   logged, but the block enters the zombie list in REQUESTED_PAUSE state.
   `cleanupZombieBlocks` eventually handles it, but only after the block
   transitions to PAUSED on its own.

5. **Scheduler `stop()` has the same REQUESTED_PAUSE issue for individual
   blocks.** The scheduler's `stop()` lifecycle callback iterates blocks and
   calls `changeStateTo(REQUESTED_STOP)` on each. Blocking blocks that are
   still in REQUESTED_PAUSE (slow to complete their pause transition) will
   fail this call. The error is logged but the block is not stopped.
