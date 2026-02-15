# Finding 009: Blocks in REQUESTED_PAUSE Cannot Be Stopped — Destructor Hangs

## Area Examined

`Scheduler.hpp:stop()` (lines 728–742), `makeZombie()` (lines 1008–1028),
`runAndWait()` ERROR cleanup (lines 416–431), and
`cleanupZombieBlocks()` (lines 924–986) — all paths that transition active
blocks toward REQUESTED_STOP.

## Observed Behavior

The state machine only allows `REQUESTED_PAUSE → PAUSED` (LifeCycle.hpp
line 83). `REQUESTED_PAUSE → REQUESTED_STOP` is invalid.

When the scheduler pauses, blocking blocks are left in REQUESTED_PAUSE
(non-blocking blocks are immediately moved to PAUSED by `pause()` at
lines 754–756). If the scheduler is then stopped (or the block is removed, or
the scheduler enters ERROR), three code paths attempt
`block->changeStateTo(REQUESTED_STOP)` — and all three fail silently:

1. **`stop()` line 737:** `block->changeStateTo(REQUESTED_STOP)` fails.
   Block stays in REQUESTED_PAUSE.

2. **`makeZombie()` line 1010:** Only handles PAUSED and RUNNING. A block
   in REQUESTED_PAUSE gets no state transition and becomes a permanent zombie.

3. **`runAndWait()` ERROR cleanup (finding 006 fix):** Same issue —
   `changeStateTo(REQUESTED_STOP)` fails for REQUESTED_PAUSE blocks.

4. **`cleanupZombieBlocks()` line 955:** Explicitly defers REQUESTED_PAUSE
   zombies ("will be moved to REQUESTED_STOP as soon as it's possible") but
   no one ever moves them.

## Issue / Risk

For blocking IO blocks, the IO thread loop (Block.hpp line 2288) runs while
`isActive()` is true. `isActive(REQUESTED_PAUSE)` returns true, so the IO
thread keeps running.

When `~Block()` runs:
1. `changeStateTo(REQUESTED_STOP)` fails (invalid from REQUESTED_PAUSE)
2. `waitOnState()` loop at line 859 waits for `isActive()` to become false
3. The IO thread keeps running (state is still REQUESTED_PAUSE)
4. **Destructor hangs forever**

Scenario:
1. Scheduler running with a blocking IO block
2. User pauses scheduler → block enters REQUESTED_PAUSE
3. User stops scheduler → `stop()` can't transition block
4. Scheduler destruction → `~Block()` hangs on `waitOnState()`

For non-blocking blocks, the impact is lower — no IO thread to hang on — but
the block is still abandoned in REQUESTED_PAUSE with its `stop()` callback
never invoked.

## Minimal Change

Before transitioning to REQUESTED_STOP, check for REQUESTED_PAUSE and
transition through PAUSED first. Apply to all four sites:

**`stop()` — line 737:**
```cpp
if (block->state() == REQUESTED_PAUSE) {
    this->emitErrorMessageIfAny("stop() REQUESTED_PAUSE -> PAUSED", block->changeStateTo(PAUSED));
}
this->emitErrorMessageIfAny("forEachBlock -> stop() -> LifecycleState", block->changeStateTo(REQUESTED_STOP));
```

**`makeZombie()` — line 1010:**
```cpp
if (block->state() == REQUESTED_PAUSE) {
    this->emitErrorMessageIfAny("makeZombie", block->changeStateTo(PAUSED));
}
if (block->state() == PAUSED || block->state() == RUNNING) {
    this->emitErrorMessageIfAny("makeZombie", block->changeStateTo(REQUESTED_STOP));
}
```

**`runAndWait()` ERROR cleanup — line 423:**
```cpp
if (block->state() == lifecycle::State::REQUESTED_PAUSE) {
    this->emitErrorMessageIfAny("runAndWait() ERROR REQUESTED_PAUSE -> PAUSED", block->changeStateTo(PAUSED));
}
this->emitErrorMessageIfAny("runAndWait() ERROR cleanup -> REQUESTED_STOP", block->changeStateTo(REQUESTED_STOP));
```

**`cleanupZombieBlocks()` — line 955:**
```cpp
case REQUESTED_PAUSE:
    this->emitErrorMessageIfAny("cleanupZombieBlocks", (*it)->changeStateTo(PAUSED));
    this->emitErrorMessageIfAny("cleanupZombieBlocks", (*it)->changeStateTo(REQUESTED_STOP));
    break;
```

## Why This Is Safe

- `REQUESTED_PAUSE → PAUSED` is a valid transition (line 83). The block's
  `pause()` callback already ran when it entered REQUESTED_PAUSE, so no
  callback fires again (the callback is on the `→ REQUESTED_PAUSE` transition,
  not the `→ PAUSED` transition).
- `PAUSED → REQUESTED_STOP` is a valid transition (line 84), triggering the
  block's `stop()` callback as expected.
- For the IO thread: once the block reaches REQUESTED_STOP, `isActive()`
  returns false, the IO thread exits its loop, and `~Block()` can complete.
- Non-blocking blocks that somehow end up in REQUESTED_PAUSE are also fixed —
  they transition cleanly through PAUSED → REQUESTED_STOP → STOPPED.
- No new states or transitions are introduced. The fix uses the existing
  state machine path: REQUESTED_PAUSE → PAUSED → REQUESTED_STOP.

## Confidence Level

**High.** The state machine constraint is explicit (line 83), the `stop()`
and `makeZombie()` code paths are directly traceable, and the `~Block()` hang
is a direct consequence of `isActive(REQUESTED_PAUSE)` returning true with no
valid path to REQUESTED_STOP.
