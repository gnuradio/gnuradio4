# GNU Radio 4 Core Runtime Audit Report

**Scope:** Core scheduler and block lifecycle interaction
**Files:** `Scheduler.hpp`, `Block.hpp`, `LifeCycle.hpp`
**Date:** 2026-02-12 (Phase 1), 2026-02-14 (Phase 2)
**Status:** 12 findings identified, all fixes applied

---

## Executive Summary

This audit examined the core runtime of GNU Radio 4 beta, focusing on the
scheduler/block lifecycle interaction, the `poolWorker` execution loop, dynamic
graph modification paths, error containment, and destructor safety.

Twelve issues were identified across two files (`Scheduler.hpp` and
`Block.hpp`), ranging from silent behavioral bugs to hard process crashes and
permanent hangs. All fixes are minimal and isolated — no new abstractions, no
architectural changes, no redesign. Every fix uses patterns already present in
the codebase and is small enough for a single short PR.

| Phase | Findings | Files Changed | Lines Changed |
|-------|----------|---------------|---------------|
| Phase 1 (committed) | 001–008 | `Scheduler.hpp`, `Block.hpp` | +47, -7 |
| Phase 2 (uncommitted) | 009–012 | `Scheduler.hpp` | +40, -5 |
| **Total** | **12** | **2 files** | **+87, -12** |

### Findings at a Glance

| # | Severity | Area | Issue | Impact |
|---|----------|------|-------|--------|
| [001](#finding-001) | Low-Med | `poolWorker` loop | Stale `activeState` delays stop/pause by up to 15 iterations | Delayed response |
| [002](#finding-002) | Medium | `runWatchDog` | Name shadowing → 100ms timeout instead of 1000ms | Watchdog exits early |
| [003](#finding-003) | Medium | `workInternal` DONE | `stop()` callback skipped when processing returns DONE | Resource leak |
| [004](#finding-004) | High | `replaceBlock` handler | Replacement block never scheduled — silently dead | Data flow stops |
| [005](#finding-005) | High | `~SchedulerBase` | Destructor hangs on PAUSED/REQUESTED_PAUSE | Program hangs |
| [006](#finding-006) | Medium | `workInternal` ERROR | Block in ERROR processes data on uninitialized resources | Corrupt output |
| [007](#finding-007) | Medium | `exchange()` | Graph exchange fails from REQUESTED_PAUSE | Exchange broken |
| [008](#finding-008) | Medium | `poolWorker` messages | Messages orphaned when runner 0 exits early | Dynamic graph broken |
| [009](#finding-009) | High | `runAndWait()` | Returns success when scheduler is in ERROR; blocks abandoned | Silent failure |
| [010](#finding-010) | High | `stop()` + workers | `stop()` transitions blocks while workers still call `work()` | Use-after-free |
| [011](#finding-011) | High | `poolWorker` noexcept | `processScheduledMessages()` throws inside `noexcept` function | `std::terminate()` |
| [012](#finding-012) | High | REQUESTED_PAUSE stop | Blocks in REQUESTED_PAUSE can't be stopped; destructor hangs | Program hangs, IO thread leak |

### Root Cause Patterns

Three systemic patterns account for the majority of findings:

1. **REQUESTED_PAUSE as a trap state** (005, 007, 012): The state machine only
   allows `REQUESTED_PAUSE → PAUSED`. Any code that attempts
   `REQUESTED_PAUSE → REQUESTED_STOP` fails silently. Six call sites were
   affected across `~SchedulerBase`, `exchange()`, `stop()`, `makeZombie()`,
   `runAndWait()` ERROR cleanup, and `cleanupZombieBlocks()`.

2. **Missing state checks** (003, 006, 009): The `workInternal()` function and
   `runAndWait()` completion path have gaps in their state-machine coverage.
   DONE bypasses `stop()`, ERROR blocks keep processing, and scheduler ERROR
   returns success to callers.

3. **Worker/lifecycle desynchronization** (010, 011): Lifecycle callbacks run on
   the caller's thread while pool workers run on the thread pool. Without
   explicit synchronization, `stop()` releases resources concurrently with
   `work()`, and exceptions propagate into `noexcept` contexts.

---

## Phase 1 Findings (Committed)

*These 8 findings were identified and fixed in commit `f268285`.*

### Finding 001

**poolWorker Stale `activeState` Cache**
Severity: Low-Medium | File: `Scheduler.hpp` | Risk of Fix: Low

The `poolWorker` loop caches the scheduler's lifecycle state in a local variable
`activeState`, but only refreshes it inside the `if (hasMessagesToProcess)`
branch — which executes once every `process_stream_to_message_ratio` iterations
(default: 16). On the other 15 iterations, the loop uses a stale value.

When an external thread requests stop or pause, pool workers continue calling
`traverseBlockListOnce` for up to 15 additional iterations before noticing.

**Fix:** Move `activeState = this->state()` outside the conditional so the
state is refreshed unconditionally every iteration.

---

### Finding 002

**Watchdog Uses Wrong Timeout Due to Name Shadowing**
Severity: Medium | File: `Scheduler.hpp` | Risk of Fix: Low

`runWatchDog(std::size_t timeOut_ms, ...)` receives the watchdog timeout
(1000ms) as a parameter, but its startup-wait phase uses the class member
`timeout_ms` (100ms) instead. The names differ only in underscore placement
(`timeout_ms` vs `timeOut_ms`), and `Annotated<T>`'s implicit conversion
silences any compiler diagnostic. The watchdog can silently exit before
monitoring starts.

**Fix:** Replace `timeout_ms` with `timeOut_ms` on the two affected lines.

---

### Finding 003

**User-Returned DONE Bypasses `stop()` Lifecycle Callback**
Severity: Medium | File: `Block.hpp` | Risk of Fix: Low

When `processBulk`/`processOne` returns `work::Status::DONE`, the block
transitions directly RUNNING → STOPPED via `setAndNotifyState()`, bypassing
`changeStateTo(REQUESTED_STOP)`. This skips the `stop()` lifecycle callback.
Blocks relying on `stop()` for cleanup (flushing buffers, closing files,
releasing hardware) silently miss that cleanup.

**Fix:** Add `changeStateTo(REQUESTED_STOP)` before `setAndNotifyState(STOPPED)`.

---

### Finding 004

**Replaced Block Never Adopted by Scheduler Workers**
Severity: High | File: `Scheduler.hpp` | Risk of Fix: Low

`propertyCallbackReplaceBlock` creates a new block but never adds it to
`_adoptionBlocks`. The replacement block exists in the graph with correct edge
references but no pool worker ever calls `work()` on it. Data flows up to its
input ports and stops. The caller receives a success notification.

**Fix:** Add the same adoption logic from `propertyCallbackEmplaceBlock`.

---

### Finding 005

**Scheduler Destructor Hangs When PAUSED or REQUESTED_PAUSE**
Severity: High | File: `Scheduler.hpp` | Risk of Fix: Low

`~SchedulerBase()` only checks `this->state() == lifecycle::RUNNING` before
stopping. If PAUSED or REQUESTED_PAUSE, no stop is initiated, and `waitDone()`
spins forever — pool workers never exit.

**Fix:** Complete REQUESTED_PAUSE → PAUSED first, then use `isActive()`.

---

### Finding 006

**`workInternal` Processes Data on Block in ERROR State**
Severity: Medium | File: `Block.hpp` | Risk of Fix: Low

`workInternal()` checks for REQUESTED_STOP and STOPPED but not ERROR. A block
in ERROR state falls through to `processBulk`/`processOne` — operating on
potentially uninitialized resources and possibly emitting corrupt data.

**Fix:** Add an ERROR state check returning DONE to cleanly stop the block.

---

### Finding 007

**`exchange()` Fails When Scheduler State Is REQUESTED_PAUSE**
Severity: Medium | File: `Scheduler.hpp` | Risk of Fix: Low

`exchange()` catches REQUESTED_PAUSE via `isActive()` but then calls
`changeStateTo(REQUESTED_STOP)` directly — invalid from REQUESTED_PAUSE.
The graph exchange returns an error and the swap never occurs.

**Fix:** Complete the pause before requesting stop.

---

### Finding 008

**Scheduler Messages Orphaned When Runner 0 Exits Early**
Severity: Medium | File: `Scheduler.hpp` | Risk of Fix: Low

Scheduler-level messages are only processed by runner 0. When runner 0's blocks
finish and it exits, remaining runners never process messages. Dynamic graph
modifications accumulate unprocessed. The internal atomic flag in
`processScheduledMessages()` already prevents concurrent execution, making the
runner-0 guard redundant.

**Fix:** Remove the runner-0 guard.

---

## Phase 2 Findings (Uncommitted)

*These 4 findings continue the audit, focusing on error containment,
worker/lifecycle synchronization, and the REQUESTED_PAUSE trap state applied to
block-level stop paths.*

### Finding 009

**`runAndWait()` Returns Success and Abandons Blocks on Scheduler ERROR**
Severity: High | File: `Scheduler.hpp` | Confidence: High

When a block's `work()` returns `work::Status::ERROR` during execution:

1. `poolWorker` transitions the **scheduler** to ERROR and exits
2. Other workers see `isActive(ERROR) == false` and exit
3. `waitDone()` returns (all workers done)
4. `runAndWait()` has no branch for ERROR → returns `{}` (**success**)

All blocks remain in RUNNING state with `stop()` callbacks never invoked.
Resources held by those callbacks are not released. Recovery via a second
`runAndWait()` call also fails — `reset()` tries RUNNING → INITIALISED, which
is invalid.

**Fix:** After `waitDone()`, insert an ERROR branch that:
- Transitions all active blocks through REQUESTED_STOP → STOPPED (same as `stop()`)
- Processes remaining messages
- Returns `std::unexpected(Error(...))` to the caller

[Full details: `audit/006-runandwait-silent-success-on-error.md`]

---

### Finding 010

**`stop()` Transitions Blocks While Pool Workers Still Access Them**
Severity: High | File: `Scheduler.hpp` | Confidence: High

`stop()` is invoked as a lifecycle callback inside
`changeStateTo(REQUESTED_STOP)`. The call sequence:

1. `setAndNotifyState(REQUESTED_STOP)` — scheduler state set atomically
2. `invokeLifecycleMethod(&TDerived::stop)` — `stop()` runs synchronously

Inside `stop()`, each block is transitioned to REQUESTED_STOP, which invokes
the block's `stop()` callback. Meanwhile, pool workers on separate threads may
still be executing `traverseBlockListOnce`, calling `work()` on those same
blocks. Workers only exit after reading the scheduler's new state, but they
check state once per iteration.

If a block's `stop()` callback releases resources (file handles, hardware, network
connections) that `workInternal()` accesses during processing → use-after-free.

Affected paths: `exchange()`, sub-scheduler stop. `runAndWait()` is safe (calls
`waitDone()` before `changeStateTo(REQUESTED_STOP)`).

**Fix:** Add `waitDone()` at the top of `stop()`. Since the scheduler state is
already REQUESTED_STOP, workers will notice and exit. The `waitDone()` call
blocks until all workers finish, ensuring no concurrent `work()` calls when
block `stop()` callbacks fire.

[Full details: `audit/007-stop-transitions-blocks-while-workers-running.md`]

---

### Finding 011

**`processScheduledMessages()` Throws Inside `noexcept` `poolWorker()`**
Severity: High | File: `Scheduler.hpp` | Confidence: High

`processScheduledMessages()` contains:

```cpp
if (this->msgOut.buffer().streamBuffer.n_readers() == 0) {
    for (const auto& msg : messagesFromChildren) {
        if (!msg.data.has_value()) {
            throw gr::exception(...);  // throws inside noexcept
        }
    }
}
```

This throws when no external listener subscribes to the scheduler's message
output port and any block emits an error message. `processScheduledMessages()`
is called from `poolWorker()` which is declared `noexcept`.

A `throw` inside a `noexcept` function invokes `std::terminate()` — the
process is killed immediately with no cleanup, no destructor calls, no error
recovery. This triggers in a common scenario: user creates a scheduler without
subscribing to messages, and any block emits an error during processing.

**Fix:** Replace `throw` with `std::println(std::cerr, ...)`, matching the
watchdog's error reporting pattern.

[Full details: `audit/008-throw-in-noexcept-poolworker.md`]

---

### Finding 012

**Blocks in REQUESTED_PAUSE Cannot Be Stopped — Destructor Hangs**
Severity: High | File: `Scheduler.hpp` | Confidence: High

The state machine only allows `REQUESTED_PAUSE → PAUSED` (LifeCycle.hpp line
83). `REQUESTED_PAUSE → REQUESTED_STOP` is invalid. Four code paths attempt
this invalid transition, and all fail silently:

| Call Site | Behavior on Failure |
|-----------|-------------------|
| `stop()` | Block stays in REQUESTED_PAUSE, `stop()` callback never fires |
| `makeZombie()` | Block becomes permanent zombie, IO thread keeps running |
| `runAndWait()` ERROR cleanup | Block abandoned in REQUESTED_PAUSE |
| `cleanupZombieBlocks()` | Zombie deferred indefinitely ("will be deleted later") |

For blocking IO blocks, this is fatal: the IO thread loop runs while
`isActive()` is true. `isActive(REQUESTED_PAUSE)` returns true. When
`~Block()` runs, it tries `changeStateTo(REQUESTED_STOP)` (fails), then
enters `waitOnState()` — waiting for `isActive()` to become false. The IO
thread keeps running. **Destructor hangs forever.**

Scenario: pause scheduler → stop scheduler → `~Block()` hangs.

*Note: The Phase 1 report flagged this as "Future Work" items 4 and 5.
This finding resolves both.*

**Fix:** Before transitioning to REQUESTED_STOP, check for REQUESTED_PAUSE
and transition through PAUSED first. Applied to all four sites:

```cpp
if (block->state() == REQUESTED_PAUSE) {
    this->emitErrorMessageIfAny("...", block->changeStateTo(PAUSED));
}
this->emitErrorMessageIfAny("...", block->changeStateTo(REQUESTED_STOP));
```

[Full details: `audit/009-requested-pause-blocks-cannot-be-stopped.md`]

---

## All Files Changed

### Phase 1 (Committed — `f268285`)

| File | Lines Added | Lines Removed | Net |
|------|------------|---------------|-----|
| `core/include/gnuradio-4.0/Scheduler.hpp` | 42 | 7 | +35 |
| `core/include/gnuradio-4.0/Block.hpp` | 5 | 0 | +5 |
| **Subtotal** | **47** | **7** | **+40** |

### Phase 2 (Uncommitted)

| File | Lines Added | Lines Removed | Net |
|------|------------|---------------|-----|
| `core/include/gnuradio-4.0/Scheduler.hpp` | 40 | 5 | +35 |
| **Subtotal** | **40** | **5** | **+35** |

### Combined Total

| | Added | Removed | Net |
|-|-------|---------|-----|
| **Grand Total** | **87** | **12** | **+75** |

---

## Test Results

All existing tests pass after both phases of fixes:

| Test Suite | Asserts | Tests | Status |
|-----------|---------|-------|--------|
| `qa_Scheduler` | 335 | 29 | All pass |
| `qa_SchedulerMessages` | 206 | 22 | All pass |
| `qa_LifeCycle` | 195 | 8 | All pass |
| **Total** | **736** | **59** | **All pass** |

---

## Observations for Future Work

The following items were noted during the audit but are outside the scope of
these fixes:

1. **`changeStateTo` uses load-then-store, not CAS.** The state machine reads
   the current state, validates the transition, then writes the new state —
   without a compare-and-swap. Under concurrent `changeStateTo` calls from
   different threads, a TOCTOU race can cause invalid transitions. This is an
   architectural concern requiring a broader design discussion.

2. **BFS/DFS schedulers silently drop unreachable blocks.** `BreadthFirst` and
   `DepthFirst` only schedule blocks reachable from source blocks. Isolated
   blocks or blocks in closed cycles without an external source are silently
   excluded. No warning is emitted.

3. **`cleanupZombieBlocks` calls `changeStateTo` under mutex.** Lifecycle
   callbacks (potentially user code) execute while `_zombieBlocksMutex` is
   held. For blocks in safe states, callbacks are no-ops. For blocks in
   transitional states, callbacks may interact with shared state.

4. **`invokeUserProvidedFunction` falls off end on exception for non-void
   return types.** The function template catches exceptions from user-provided
   lambdas but doesn't return a value in the catch path. Current call sites all
   use void lambdas (they assign to captured references), so UB doesn't
   manifest, but the template is unsafe for non-void return types.

5. **`consumeReaders` failure after output published.** In `workInternal()`,
   output is published (line 2183) before input consumption (line 2189). If
   `consumeReaders` fails, output has already been committed to downstream
   blocks. The scheduler enters ERROR and stops, so no duplicate processing
   occurs, but one chunk of output was generated from "unconsumed" input.

6. **`exchange()` calls `reset()` on old graph before swap.** When the
   scheduler was active, `reset()` is called explicitly (line 248) on the old
   graph, then again via lifecycle callback when `changeStateTo(INITIALISED)`
   fires (line 263) on the new graph. The first call unnecessarily modifies the
   old graph (blocks → INITIALISED, edges disconnected) before returning it.

7. **`makeAllZombies()` may still have REQUESTED_PAUSE issues.** While
   `makeZombie()` and `stop()` are now fixed (Finding 012), `makeAllZombies()`
   has its own inline state-transition logic that independently handles
   REQUESTED_PAUSE. It should be reviewed for consistency.
