# Finding 013: makeAllZombies attempts invalid REQUESTED_PAUSE → REQUESTED_STOP transition

## Invariant
A block in `REQUESTED_PAUSE` must transition through `PAUSED` before it can reach `REQUESTED_STOP`. This is enforced by `isValidTransition()` in `LifeCycle.hpp:83`.

## Code Locus
`Scheduler.hpp` — `makeAllZombies()`, previously lines 1047–1050.

```cpp
case RUNNING:
case REQUESTED_PAUSE:
case PAUSED: //
    this->emitErrorMessageIfAny("makeAllZombies", block->changeStateTo(REQUESTED_STOP));
    break;
```

## Possible Violation
Finding 009 fixed four sites where `REQUESTED_PAUSE → REQUESTED_STOP` was attempted directly:
1. `stop()` callback
2. `makeZombie()`
3. ERROR cleanup in `cleanupZombieBlocks()`
4. `cleanupZombieBlocks()` main path

This fifth site — `makeAllZombies()` — was missed. When triggered (e.g., via "set grc yaml" message in `propertyCallbackGraphGRC`), any block in `REQUESTED_PAUSE` fails the transition silently. The block is moved to `_zombieBlocks` in `REQUESTED_PAUSE` state, where `cleanupZombieBlocks()` will try to transition it through `PAUSED → REQUESTED_STOP`, but only if a pool worker owns that block. If no worker claims it, the zombie persists indefinitely.

## Fix
Transition through `PAUSED` first, matching the pattern at the other four sites:

```cpp
case REQUESTED_PAUSE: // REQUESTED_PAUSE → REQUESTED_STOP is invalid; go through PAUSED first
    this->emitErrorMessageIfAny("makeAllZombies REQUESTED_PAUSE -> PAUSED", block->changeStateTo(PAUSED));
    [[fallthrough]];
case RUNNING:
case PAUSED: //
    this->emitErrorMessageIfAny("makeAllZombies", block->changeStateTo(REQUESTED_STOP));
    break;
```

## Why Safe
- `REQUESTED_PAUSE → PAUSED` is always valid (LifeCycle.hpp:83)
- `PAUSED → REQUESTED_STOP` is always valid (LifeCycle.hpp:84)
- Matches the exact pattern used at the four sites already fixed by finding 009
- `makeAllZombies()` runs under `_zombieBlocksMutex`, and the block state transitions are atomic

## CI Impact
All existing scheduler tests (qa_Scheduler: 29 tests, qa_SchedulerMessages: 22 tests) continue to pass.
