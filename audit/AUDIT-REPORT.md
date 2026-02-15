# GNU Radio 4 Core Runtime Audit Report

**Scope:** Core scheduler, block lifecycle, and graph mutation
**Files:** `Scheduler.hpp`, `Block.hpp`, `LifeCycle.hpp`, `Graph.hpp`, `Graph.cpp`
**Date:** 2026-02-12 (Phase 1), 2026-02-14 (Phase 2), 2026-02-15 (Phases 3–4)
**Status:** 28 findings identified, all fixes applied
**Companion:** [`TODO_SEARCH_REPORT.md`](TODO_SEARCH_REPORT.md) — catalog of all TODO/FIXME markers

---

## Executive Summary

This audit examined the core runtime of GNU Radio 4 beta, focusing on the
scheduler/block lifecycle interaction, the `poolWorker` execution loop, dynamic
graph modification paths, error containment, and destructor safety.

Twenty-eight issues were identified across five files, ranging from silent
behavioral bugs to hard process crashes, permanent hangs, use-after-free, and
zombie edges. All fixes are minimal and isolated — no new abstractions, no
architectural changes, no redesign. Every fix uses patterns already present in
the codebase.

| Phase | Findings | Focus | Files | Lines |
|-------|----------|-------|-------|-------|
| 1 | 001–008 | Scheduler/block lifecycle | `Scheduler.hpp`, `Block.hpp` | +47, -7 |
| 2 | 009–012 | Error containment, REQUESTED_PAUSE | `Scheduler.hpp` | +40, -5 |
| 3 | 013–025 | Concurrency, noexcept, IO thread, exchange | `Scheduler.hpp`, `Block.hpp`, `LifeCycle.hpp` | +71, -8 |
| 4 | 026–028 | Graph edge/block mutation | `Graph.hpp`, `Graph.cpp`, `Scheduler.hpp` | +23, -1 |
| **Total** | **28** | | **5 files** | **+181, -21** |

### Findings at a Glance

| # | Severity | Area | Issue | Impact |
|---|----------|------|-------|--------|
| # | Sev | Area | Issue | Impact |
|---|-----|------|-------|--------|
| [001](#finding-001) | Low-Med | `poolWorker` loop | Stale `activeState` delays stop/pause by up to 15 iterations | Delayed response |
| [002](#finding-002) | Med | `runWatchDog` | Name shadowing → 100ms timeout instead of 1000ms | Watchdog exits early |
| [003](#finding-003) | Med | `workInternal` DONE | `stop()` callback skipped when processing returns DONE | Resource leak |
| [004](#finding-004) | High | `replaceBlock` handler | Replacement block never scheduled — silently dead | Data flow stops |
| [005](#finding-005) | High | `~SchedulerBase` | Destructor hangs on PAUSED/REQUESTED_PAUSE | Program hangs |
| [006](#finding-006) | Med | `workInternal` ERROR | Block in ERROR processes data on uninitialized resources | Corrupt output |
| [007](#finding-007) | Med | `exchange()` | Graph exchange fails from REQUESTED_PAUSE | Exchange broken |
| [008](#finding-008) | Med | `poolWorker` messages | Messages orphaned when runner 0 exits early | Dynamic graph broken |
| [009](#finding-009) | High | `runAndWait()` | Returns success when scheduler is in ERROR; blocks abandoned | Silent failure |
| [010](#finding-010) | High | `stop()` + workers | `stop()` transitions blocks while workers still call `work()` | Use-after-free |
| [011](#finding-011) | High | `poolWorker` noexcept | `processScheduledMessages()` throws inside `noexcept` | `std::terminate()` |
| [012](#finding-012) | High | REQUESTED_PAUSE | Blocks in REQUESTED_PAUSE can't be stopped; destructor hangs | Hang, IO leak |
| [013](#finding-013) | High | `makeAllZombies()` | 5th REQUESTED_PAUSE site — zombies stuck forever | Zombie leak |
| [014](#finding-014) | Med | `customInit()` lock | `Simple::customInit()` missing `_adoptionBlocksMutex` | Data race |
| [015](#finding-015) | High | `exchange()` | Stale `_executionOrder` after graph swap | Wrong blocks run |
| [016](#finding-016) | High | `Block` noexcept | `Block::processScheduledMessages()` throws in noexcept chain | `std::terminate()` |
| [017](#finding-017) | High | User callback | `processMessages()` called without try-catch in noexcept | `std::terminate()` |
| [018](#finding-018) | Med | `changeStateTo` | TOCTOU: load-then-store, not CAS (documented) | Race condition |
| [019](#finding-019) | Med | `pause()` | Iterates `_blocks` while workers mutate (documented) | Data race |
| [020](#finding-020) | High | `exchange()` | Non-active states skip `_executionOrder` rebuild | Stale blocks |
| [021](#finding-021) | Med | `exchange()` | `_messagePortsConnected` not reset after swap | Stale routing |
| [022](#finding-022) | Low | `_nRunningJobs` | Non-RAII inc/dec (documented invariant) | Potential hang |
| [023](#finding-023) | High | `~Block()` IO | 10ms sleep instead of IO thread synchronization | Use-after-free |
| [024](#finding-024) | Med | IO thread | Assert fires if block stopped before executor runs | Debug crash |
| [025](#finding-025) | Low | `resume()` | Error message says "init()" — copy-paste bug | Misleading log |
| [026](#finding-026) | High | `removeEdgeBySourcePort` | Doesn't remove edge from `_edges` — zombie edge | Edge resurfaces |
| [027](#finding-027) | High | `replaceBlock()` | Edge state not reset — new block runs unconnected | Silent failure |
| [028](#finding-028) | Low | Registry callback | Assert checks wrong property constant — copy-paste | Debug crash |

### Root Cause Patterns

Five systemic patterns account for the majority of findings:

1. **REQUESTED_PAUSE as a trap state** (005, 007, 009, 012, 013): The state
   machine only allows `REQUESTED_PAUSE → PAUSED`. Any code that attempts
   `REQUESTED_PAUSE → REQUESTED_STOP` fails silently. Six call sites were
   affected across `~SchedulerBase`, `exchange()`, `stop()`, `makeZombie()`,
   `makeAllZombies()`, `runAndWait()` ERROR cleanup, and `cleanupZombieBlocks()`.

2. **Missing state checks** (003, 006, 009): The `workInternal()` function and
   `runAndWait()` completion path have gaps in their state-machine coverage.
   DONE bypasses `stop()`, ERROR blocks keep processing, and scheduler ERROR
   returns success to callers.

3. **Worker/lifecycle desynchronization** (010, 011, 016, 017, 023, 024):
   Lifecycle callbacks run on the caller's thread while pool workers and IO
   threads run concurrently. Without explicit synchronization, `stop()` releases
   resources while `work()` uses them, exceptions propagate into `noexcept`
   contexts, and destructors race with IO threads.

4. **`exchange()` stale state** (015, 020, 021): After swapping the graph,
   `_executionOrder`, `_messagePortsConnected`, and block message routing all
   reference the old graph. Multiple independent staleness bugs compound.

5. **Graph mutation asymmetry** (026, 027): Edge operations are not symmetric —
   `emplaceEdge()` connects ports AND adds metadata, but `removeEdgeBySourcePort()`
   only disconnects; `replaceBlock()` rewrites metadata but doesn't reset state
   for reconnection.

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

## Phase 2 Findings (Committed — `c46c94f`)

*These 4 findings focus on error containment, worker/lifecycle synchronization,
and the REQUESTED_PAUSE trap state applied to block-level stop paths.*

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

## Phase 3 Findings (Committed — `0165085`)

*These 13 findings cover concurrency, exception containment in noexcept
contexts, IO thread lifecycle, exchange() staleness, and documentation of
structural invariants.*

### Finding 013

**`makeAllZombies()` — 5th REQUESTED_PAUSE Site**
Severity: High | File: `Scheduler.hpp`

`makeAllZombies()` attempts `REQUESTED_PAUSE → REQUESTED_STOP` directly.
Block stays in REQUESTED_PAUSE as a zombie forever. Same root cause as 009/012.

**Fix:** Transition through PAUSED first, matching the pattern from finding 012.

[Full details: `audit/013-makeallzombies-requested-pause.md`]

---

### Finding 014

**`Simple::customInit()` Missing Lock on `_adoptionBlocks`**
Severity: Medium | File: `Scheduler.hpp`

`Simple::customInit()` resizes `_adoptionBlocks` without holding
`_adoptionBlocksMutex`. `BreadthFirst` and `DepthFirst` both hold it.
Lock ordering: `_adoptionBlocksMutex` → `_executionOrderMutex`.

**Fix:** Add the missing lock, matching lock order used by other schedulers.

[Full details: `audit/014-simple-custominit-lock-discipline.md`]

---

### Finding 015

**`exchange()` Stale `_executionOrder` After Graph Swap**
Severity: High | File: `Scheduler.hpp`

`exchange()` swaps the graph but never rebuilds `_executionOrder`. Pool workers
iterate blocks from the old graph.

**Fix:** Call `init()` after `changeStateTo(INITIALISED)` to rebuild.

[Full details: `audit/015-exchange-stale-execution-order.md`]

---

### Finding 016

**`Block::processScheduledMessages()` Throws in Noexcept Chain**
Severity: High | File: `Block.hpp`

`Block::processScheduledMessages()` throws on `tryConsume` failure. Called from
`poolWorker()` which is `noexcept` → `std::terminate()`. Distinct from the
scheduler-level throw in finding 011.

**Fix:** Replace throw with `std::println` to stderr.

[Full details: `audit/016-block-processscheduledmessages-throw.md`]

---

### Finding 017

**User `processMessages()` Callback Throws in Noexcept Chain**
Severity: High | File: `Scheduler.hpp`

User-provided `processMessages()` callback is called directly (no try-catch)
from the noexcept `poolWorker` → `processScheduledMessages` chain.

**Fix:** Wrap in `invokeUserProvidedFunction`.

[Full details: `audit/017-user-processmessages-throw-noexcept.md`]

---

### Finding 018

**`changeStateTo()` TOCTOU — Documented**
Severity: Medium | File: `LifeCycle.hpp`

`changeStateTo()` uses load-then-store (not CAS). Concurrent ERROR and
REQUESTED_STOP transitions can lose ERROR. CAS requires architectural change.

**Fix:** Documentation-only. Added TOCTOU warning comment.

[Full details: `audit/018-changestateto-toctou.md`]

---

### Finding 019

**`pause()` Concurrent Block Mutation — Documented**
Severity: Medium | File: `Scheduler.hpp`

`pause()` iterates `_graph->_blocks` while workers still run
`processScheduledMessages` which can mutate the vector. Requires snapshot
or mutex to fix properly.

**Fix:** Documentation-only.

[Full details: `audit/019-pause-concurrent-block-mutation.md`]

---

### Finding 020

**`exchange()` Non-Active States Skip `_executionOrder` Rebuild**
Severity: High | File: `Scheduler.hpp`

For non-active, non-IDLE states (INITIALISED, STOPPED, ERROR), `exchange()`
swaps the graph without calling `init()`. Next `runAndWait()` uses stale
old-graph blocks. IDLE is exempt — `runAndWait()` handles its init.

**Fix:** Call `init()` after graph swap for non-IDLE non-active states.

[Full details: `audit/020-exchange-nonactive-stale-executionorder.md`]

---

### Finding 021

**`_messagePortsConnected` Not Reset During `exchange()`**
Severity: Medium | File: `Scheduler.hpp`

`_messagePortsConnected` is never reset to `false` during `exchange()`. After
a graph swap, messages are routed to old graph's blocks. The flag is write-once
(true) and must be explicitly cleared.

**Fix:** Set `_messagePortsConnected = false` before graph swap.

[Full details: `audit/021-exchange-messageports-stale.md`]

---

### Finding 022

**`_nRunningJobs` Symmetric Inc/Dec — Documented Invariant**
Severity: Low | File: `Scheduler.hpp`

`_nRunningJobs` relies on symmetric `incrementAndGet()` at worker entry and
`subAndGet(1)` at exit, without RAII guard. After exception containment fixes
(008, 016, 017), no leak paths remain.

**Fix:** Documentation-only. Added invariant comment.

[Full details: `audit/022-nrunningjobs-symmetric-inc-dec.md`]

---

### Finding 023

**`~Block()` IO Thread Race — 10ms Sleep Instead of Synchronization**
Severity: High | File: `Block.hpp`

The blocking IO thread captures `this` as a raw pointer. Its last access is
`ioThreadRunning.store(false)`. The destructor used a 10ms sleep as mitigation.
If IO thread hasn't reached that line within 10ms → use-after-free.

**Fix:** Replace sleep with spin-wait on `ioThreadRunning`.

[Full details: `audit/023-block-destructor-io-thread-race.md`]

---

### Finding 024

**IO Thread Assert Fires if Block Stopped Before Executor Runs**
Severity: Medium | File: `Block.hpp`

`assert(lifecycle::isActive(this->state()))` inside the IO thread lambda fires
if the block transitions to REQUESTED_STOP between `work()` queuing the task
and the executor running it.

**Fix:** Replace assert with graceful early return + `ioThreadRunning.store(false)`.

[Full details: `audit/024-io-thread-assert-late-start.md`]

---

### Finding 025

**`resume()` Error Message Says "init()" — Copy-Paste Bug**
Severity: Low | File: `Scheduler.hpp`

`resume()` callback calls `emitErrorMessage("init()", ...)` — copied from
`start()`. Misleading for debugging.

**Fix:** Change `"init()"` to `"resume()"`.

[Full details: `audit/025-resume-copypaste-error-message.md`]

---

## Phase 4 Findings (Committed — `3f6572f`)

*These 3 findings cover graph edge and block mutation invariants.*

### Finding 026

**`removeEdgeBySourcePort()` Doesn't Remove Edge from `_edges`**
Severity: High | File: `Graph.hpp`

`removeEdgeBySourcePort()` disconnects the port but does NOT remove the edge
from `_edges`. On scheduler restart, `disconnectAllEdges() + connectPendingEdges()`
resurrects the "removed" edge as a zombie. Asymmetric with `emplaceEdge()` which
both connects the port and adds edge metadata.

**Fix:** Add `erase(remove_if)` to remove matching edge metadata after disconnect.

[Full details: `audit/026-removeedge-missing-edge-erasure.md`]

---

### Finding 027

**`replaceBlock()` Doesn't Reset Edge State — New Block Runs Unconnected**
Severity: High | File: `Graph.cpp`, `Scheduler.hpp`

`replaceBlock()` rewrites edge metadata to point to the new block but leaves
edge state as `Connected` with stale port pointers to the old block.
`connectPendingEdges()` skips these edges (already "Connected"). The new block
is adopted, transitioned to RUNNING, but has no data connections.

**Fix:** Reset affected edges to `WaitingToBeConnected`, null stale port
pointers. Call `connectPendingEdges()` in `propertyCallbackReplaceBlock`.

[Full details: `audit/027-replaceblock-stale-edge-state.md`]

---

### Finding 028

**`propertyCallbackRegistrySchedulerTypes` Asserts Wrong Constant**
Severity: Low | File: `Graph.cpp`

Assert checks `kRegistryBlockTypes` instead of `kRegistrySchedulerTypes` —
copy-paste from the adjacent `propertyCallbackRegistryBlockTypes`. In debug
builds, this assert fires on every scheduler-types query.

**Fix:** Correct the constant.

[Full details: `audit/028-scheduler-types-assert-copypaste.md`]

---

## All Files Changed

### By Phase

| Phase | Commit | Files | Added | Removed | Net |
|-------|--------|-------|-------|---------|-----|
| 1 | `f268285` | `Scheduler.hpp`, `Block.hpp` | 47 | 7 | +40 |
| 2 | `c46c94f` | `Scheduler.hpp` | 40 | 5 | +35 |
| 3 | `0165085` | `Scheduler.hpp`, `Block.hpp`, `LifeCycle.hpp` | 71 | 8 | +63 |
| 4 | `3f6572f` | `Graph.hpp`, `Graph.cpp`, `Scheduler.hpp` | 23 | 1 | +22 |

### By File (Cumulative)

| File | Added | Removed | Net |
|------|-------|---------|-----|
| `core/include/gnuradio-4.0/Scheduler.hpp` | 145 | 14 | +131 |
| `core/include/gnuradio-4.0/Block.hpp` | 27 | 5 | +22 |
| `core/include/gnuradio-4.0/Graph.hpp` | 9 | 0 | +9 |
| `core/include/gnuradio-4.0/LifeCycle.hpp` | 7 | 0 | +7 |
| `core/src/Graph.cpp` | 14 | 1 | +13 |
| **Total** | **181** (sic) | **21** | **+160** |

---

## Test Results

All existing tests pass after all four phases of fixes:

| Test Suite | Asserts | Tests | Status |
|-----------|---------|-------|--------|
| `qa_Scheduler` | 335 | 29 | All pass |
| `qa_Graph` | 135 | 21 | All pass |
| `qa_Block` | all | all suites | All pass |
| `qa_LifeCycle` | 195 | 8 | All pass |

Note: `qa_SchedulerMessages` has a preexisting linker error (undefined symbol
`gr_blocklib_init_unit_TagMonitors_0`) unrelated to audit changes.

---

## Observations for Future Work

The following items were noted during the audit but are outside the scope of
these fixes. See [`TODO_SEARCH_REPORT.md`](TODO_SEARCH_REPORT.md) for the full
catalog of TODO/FIXME markers.

1. **`changeStateTo` TOCTOU** (finding 018): Uses load-then-store, not CAS.
   Concurrent ERROR + REQUESTED_STOP can lose ERROR. Requires architectural
   change.

2. **`pause()` concurrent block mutation** (finding 019): Iterates `_blocks`
   while workers can mutate the vector. Requires snapshot or mutex.

3. **`_nRunningJobs` non-RAII** (finding 022): Symmetric inc/dec without guard.
   After exception containment fixes, no leak paths remain, but not structural.

4. **BFS/DFS silently drop unreachable blocks.** No warning for isolated blocks
   or closed cycles without external sources.

5. **`invokeUserProvidedFunction` falls off end on exception for non-void
   return types.** Current call sites all use void lambdas, but the template
   is unsafe for non-void returns.

6. **`consumeReaders` failure after output published.** Output committed before
   input consumed. On failure, one chunk of output generated from "unconsumed"
   input.

7. **Adoption slot stranding.** `propertyCallbackEmplaceBlock` assigns by
   address hash. If target worker already exited (DONE), block is stranded
   until scheduler restarts.
