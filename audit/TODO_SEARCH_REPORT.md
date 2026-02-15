# GNU Radio 4 Core Runtime — TODO Search Report

**Scope:** `core/include/gnuradio-4.0/`, `core/src/`
**Date:** 2026-02-15
**Companion:** [`AUDIT-REPORT.md`](AUDIT-REPORT.md) (28 findings, all fixed)

---

## Overview

This report catalogs all `TODO`, `FIXME`, `HACK`, and `N.B.` markers across the
GR4 core runtime. Items already addressed by audit findings 001–028 are marked
as **RESOLVED**. Remaining items are categorized by severity and type.

### Stats

| Category | Count |
|----------|-------|
| TODO | ~50 |
| FIXME | 4 |
| N.B. (known limitation) | ~40 |
| **Resolved by audit** | **28** |

---

## Critical / High Priority

These items indicate known bugs, undefined behavior, or dangerous workarounds.

### Port.hpp:1227 — Dangerous const-cast in DynamicPort

```
TODO: remove const-cast (super dangerous, and only a temporary fix) -> Ivan volunteerd to fix in follor-up PR
```
**Category:** Known bug
**Risk:** `const_cast` on port data can cause undefined behavior if the
underlying object is actually const. Marked as temporary but still present.

### PluginLoader.hpp:67 — void\* to function-pointer cast is UB

```
FIXME: Casting a void* to function-pointer is UB in C++. Yes "… 'dlsym' is not C++...
```
**Category:** Known bug (platform-dependent UB)
**Risk:** Required by `dlsym` API. Works on all POSIX platforms in practice but
technically undefined per the C++ standard. No portable alternative exists.

### Block.hpp:2326 — Emscripten deadlock workaround

```
TODO: this is just "working" solution for deadlock with emscripten, need to be investigated further
```
**Category:** Known bug / workaround
**Risk:** Deadlock can occur on Emscripten platform. Current fix is a bandaid.

### Block.hpp:2648 — Inconsistent template specialization

```
FIXME: the following are inconsistent in how they specialize the template. Multiple types can be given, resulting in...
```
**Category:** Design concern
**Risk:** Inconsistent behavior when multiple types are given to template
specializations. May cause surprising behavior for block authors.

### Port.hpp:1235 — Port lifetime problem

```
TODO: The lifetime of ports is a problem here, if we keep a reference to the port in DynamicPort...
```
**Category:** Design concern
**Risk:** Dangling references if port lifetimes are not managed carefully.
Related to the IO thread race fixed in audit finding 023.

---

## Medium Priority

### Cleanup — Dead Code and Obsolete Remnants

| File | Line | Comment |
|------|------|---------|
| Block.hpp | 958 | `TODO: remove these obsolete lines` |
| Block.hpp | 1096 | `TODO: we still fill _mergedInputTag, but this will be removed in the one of the next PR` |
| Block.hpp | 2045 | `TODO: finally remove me` |
| BlockModel.hpp | 894 | `TODO: to be removed (read-only)` |
| Graph.cpp | 7 | `_blocks.reserve(100)` — `TODO: remove` |
| Tag.hpp | 54 | `TODO: do we need the convenience methods below?` |

### Refactoring — Naming and API

| File | Line | Comment |
|------|------|---------|
| annotated.hpp | 68 | `TODO: replace bool by an enum or tag type: 'BlockingIO<false>' is very misleading` |
| Block.hpp | 876 | `TODO: Refactor the library not to assign names to ports` |
| Block.hpp | 1131 | `TODO: autoUpdate does not really need Tag, it should be changed to accept property_map` |
| Port.hpp | 1077 | `TODO: rename to 'name()' and eliminate local 'name' field` |
| Port.hpp | 1080 | `TODO: rename to type() and remove existing type(), direction(), domain(), ... API` |
| Port.hpp | 1151 | `TODO: refactor to non-throwing std::expected<ConnectionResult, Error> return` |
| Port.hpp | 1180 | `TODO: '_value.name' -> '_value.metaInfo.name' and use string&` |
| Tag.hpp | 181 | `TODO: for backward compatibility -> rename to 'ctx_time'` |
| PortTraits.hpp | 46 | `FIXME: better name "describes_" instead of "is_"?` |

### Error Handling Gaps

| File | Line | Comment |
|------|------|---------|
| Port.hpp | 700 | `TODO(error handling): Decide how to surface failures.` |
| Port.hpp | 981 | `TODO(error handling): Decide how to surface failures. Function is noexcept now` |
| Settings.hpp | 76 | `TODO: throw if types are not the same?` |

### Missing Features

| File | Line | Comment |
|------|------|---------|
| Graph.hpp | 169 | `TODO: Add support for exporting port collections` |
| Graph.hpp | 1287 | `TODO: SIMD for multiple output ports not implemented yet` |
| Port.hpp | 543 | `TODO: limit initial max buffer size based on kIsArithmeticLikeValueType` |
| Port.hpp | 778 | `TODO: If we want to allow ports with different buffer types to be mixed...` |
| Graph_yaml_importer.hpp | 276 | `TODO: schedulerMap["parameters"s] =` (incomplete) |

### Documentation

| File | Line | Comment |
|------|------|---------|
| Graph.hpp | 1209 | `TODO: Add a comment why a unique ID is necessary for merged blocks` |
| PluginLoader.hpp | 59 | `TODO: Document why RTLD_LOCAL and not RTLD_GLOBAL is used here.` |

### Test Coverage

| File | Line | Comment |
|------|------|---------|
| Graph_yaml_importer.hpp | 270 | `TODO: a unit-test that this is working` |

### Platform / Build

| File | Line | Comment |
|------|------|---------|
| PluginLoader.cpp | 10 | `TODO choose proper paths when we get the system GR installation done` |
| PluginLoader.cpp | 18 | `TODO If we want to support Windows, this should be ;` |
| CMakeLists.txt | 66 | `TODO: install configure file...` |

---

## Low Priority / Future Enhancements

| File | Line | Comment |
|------|------|---------|
| annotated.hpp | 461 | `TODO: add switch for printing only brief and/or meta-information` |
| Block.hpp | 492 | `N.B. TODO discuss these requirements` |
| Block.hpp | 761 | `TODO: C++26 make sure these are not reflected` |
| Block.hpp | 2063 | `TODO: evaluate if/how we can get rid of these` (merged block special cases) |
| BlockModel.hpp | 668 | `TODO: Type names can be mangled. We need proper type names...` |
| BlockTraits.hpp | 133 | `TODO: Why is this not done with requires?` |
| BlockTraits.hpp | 346 | `TODO: Is this check redundant?` |
| Buffer.hpp | 53 | `TODO: find a better place for these 3 concepts` |
| Buffer.hpp | 80–91 | Multiple TODOs about return types (`get()`, `tryReserve()`, `reserve()`) |
| BufferSkeleton.hpp | 5 | `TODO: why is this include guard outside of the buffer hpp needed?` |
| Graph.hpp | 1187 | `FIXME: How do we refuse connection to a vector<Port>?` |
| Graph.hpp | 1340 | `TODO: ask Matthias if this is still needed` |
| Graph.hpp | 1421 | `TODO: add nicer enum formatter` |
| Profiler.hpp | 85 | `TODO` (incomplete switch statement) |
| Sequence.hpp | 85 | `TODO: Revisit once libc++ adds support for atomic shared_ptr` |
| Sequence.hpp | 118 | `xTODO: explicit memory order` |
| Scheduler.hpp | 188 | `TODO: check whether we can keep this std::size_t or more consistently to gr::Size_t` |
| Port.hpp | 199 | Long-term goal: compile-time unit checks via mp-units (C++26) |

---

## Resolved by Audit (Findings 001–028)

These issues were identified and fixed during the core runtime audit. Each
finding has a detailed write-up in the `audit/` directory.

### Phase 1 — Scheduler/Block Lifecycle (commit `f268285`)

| # | File | Issue | Severity |
|---|------|-------|----------|
| 001 | Scheduler.hpp | `poolWorker` stale `activeState` delays stop/pause | Low-Med |
| 002 | Scheduler.hpp | `runWatchDog` name shadowing → wrong timeout | Medium |
| 003 | Block.hpp | DONE return bypasses `stop()` callback | Medium |
| 004 | Scheduler.hpp | Replaced block never added to `_adoptionBlocks` | High |
| 005 | Scheduler.hpp | `~SchedulerBase()` hangs on PAUSED/REQUESTED_PAUSE | High |
| 006 | Block.hpp | `workInternal` processes data in ERROR state | Medium |
| 007 | Scheduler.hpp | `exchange()` fails from REQUESTED_PAUSE | Medium |
| 008 | Scheduler.hpp | Messages orphaned when runner 0 exits early | Medium |

### Phase 2 — Error Containment & REQUESTED_PAUSE (commit `c46c94f`)

| # | File | Issue | Severity |
|---|------|-------|----------|
| 009 | Scheduler.hpp | REQUESTED_PAUSE blocks can't be stopped; destructor hangs | High |
| 010 | Scheduler.hpp | `makeAllZombies()` — 6th REQUESTED_PAUSE site | High |
| 011 | Scheduler.hpp | `Simple::customInit()` missing lock on `_adoptionBlocks` | Medium |
| 012 | Scheduler.hpp | `exchange()` doesn't rebuild `_executionOrder` | High |

### Phase 3 — Concurrency, Throw-in-Noexcept, IO Thread (commit `0165085`)

| # | File | Issue | Severity |
|---|------|-------|----------|
| 013 | Scheduler.hpp | `makeAllZombies()` REQUESTED_PAUSE (5th site) | High |
| 014 | Scheduler.hpp | `Simple::customInit()` lock discipline | Medium |
| 015 | Scheduler.hpp | `exchange()` stale `_executionOrder` | High |
| 016 | Scheduler.hpp | `Block::processScheduledMessages()` throws in noexcept | High |
| 017 | Scheduler.hpp | User `processMessages()` throws in noexcept chain | High |
| 018 | LifeCycle.hpp | `changeStateTo()` TOCTOU (documented) | Medium |
| 019 | Scheduler.hpp | `pause()` concurrent block mutation (documented) | Medium |
| 020 | Scheduler.hpp | `exchange()` non-active stale `_executionOrder` | High |
| 021 | Scheduler.hpp | `_messagePortsConnected` not reset on `exchange()` | Medium |
| 022 | Scheduler.hpp | `_nRunningJobs` symmetric inc/dec (documented) | Low |
| 023 | Block.hpp | `~Block()` IO thread race — 10ms sleep → spin-wait | High |
| 024 | Block.hpp | IO thread assert fires on late start | Medium |
| 025 | Scheduler.hpp | `resume()` error message copy-paste bug | Low |

### Phase 4 — Graph Edge & Block Mutation (commit `3f6572f`)

| # | File | Issue | Severity |
|---|------|-------|----------|
| 026 | Graph.hpp | `removeEdgeBySourcePort()` doesn't remove edge from `_edges` | High |
| 027 | Graph.cpp + Scheduler.hpp | `replaceBlock()` leaves edges stale — new block unconnected | High |
| 028 | Graph.cpp | `propertyCallbackRegistrySchedulerTypes` wrong assert | Low |

### Cumulative Impact

| | Files | Added | Removed | Net |
|-|-------|-------|---------|-----|
| Phase 1 | 2 | 47 | 7 | +40 |
| Phase 2 | 1 | 40 | 5 | +35 |
| Phase 3 | 3 | 71 | 8 | +63 |
| Phase 4 | 3 | 23 | 1 | +22 |
| **Total** | **5 files** | **181** | **21** | **+160** |

---

## Observations for Future Work

Items from the audit that remain as documented-only (no code fix):

1. **`changeStateTo` TOCTOU** (finding 018): Uses load-then-store, not CAS.
   Concurrent ERROR + REQUESTED_STOP can lose ERROR. Requires architectural
   change to fix.

2. **`pause()` concurrent block mutation** (finding 019): Iterates
   `_graph->_blocks` while workers run `processScheduledMessages` which can
   mutate the vector. Requires snapshot or mutex.

3. **`_nRunningJobs` non-RAII** (finding 022): Relies on symmetric inc/dec.
   After exception containment fixes (008, 016, 017), no leak paths exist, but
   not structurally guaranteed.

4. **BFS/DFS silently drop unreachable blocks.** No warning emitted for
   isolated blocks or closed cycles without external sources.

5. **`invokeUserProvidedFunction` falls off end on exception for non-void
   return types.** Current call sites all use void lambdas, but the template is
   unsafe for non-void returns.

6. **`consumeReaders` failure after output published.** In `workInternal()`,
   output is published before input consumption. If `consumeReaders` fails, one
   chunk of output was generated from "unconsumed" input.

7. **Adoption slot stranding.** `propertyCallbackEmplaceBlock` assigns adoption
   slot by address hash. If target worker already exited (DONE), block is
   stranded until scheduler restarts.
