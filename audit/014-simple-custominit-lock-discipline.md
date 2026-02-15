# Finding 014: Simple::customInit() modifies _adoptionBlocks without holding _adoptionBlocksMutex

## Invariant
`_adoptionBlocks` must only be read or written under `_adoptionBlocksMutex`. This is the lock discipline established by `adoptBlocks()`, `propertyCallbackEmplaceBlock`, `propertyCallbackReplaceBlock`, `makeZombie()`, and the `customInit()` implementations of BreadthFirst and DepthFirst.

## Code Locus
`Scheduler.hpp` — `Simple::customInit()`, previously lines 1214–1218.

```cpp
std::lock_guard lock(this->_executionOrderMutex);
this->_adoptionBlocks.clear();      // ← _adoptionBlocksMutex NOT held
this->_adoptionBlocks.resize(n_batches);
```

Compare with BreadthFirst::customInit() and DepthFirst::customInit(), which both acquire `_adoptionBlocksMutex` first:
```cpp
std::lock_guard guard(this->_adoptionBlocksMutex);
std::lock_guard lock(this->_executionOrderMutex);
```

## Possible Violation
If a message arrives during the IDLE→INITIALISED transition (before blocks-to-RUNNING in start()), `propertyCallbackEmplaceBlock` could read `_adoptionBlocks.size()` under `_adoptionBlocksMutex` while `Simple::customInit()` is clearing and resizing the same vector without that lock. This is a data race (undefined behavior).

In practice, this is unlikely because `propertyCallbackEmplaceBlock` guards on `lifecycle::isActive(this->state())` which is false during INITIALISED. But the invariant should be structurally enforced, not rely on state-machine timing.

## Fix
Add `_adoptionBlocksMutex` acquisition before `_executionOrderMutex`, matching BreadthFirst/DepthFirst:

```cpp
std::lock_guard guard(this->_adoptionBlocksMutex);
std::lock_guard lock(this->_executionOrderMutex);
```

Lock order is `_adoptionBlocksMutex` → `_executionOrderMutex`, consistent with all existing sites.

## Why Safe
- Matches lock discipline already used by BreadthFirst and DepthFirst
- Lock ordering is consistent — no deadlock risk
- No behavioral change; only adds structural safety

## CI Impact
All existing scheduler tests continue to pass.
