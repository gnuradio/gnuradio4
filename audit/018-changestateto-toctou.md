# Finding 018: changeStateTo() uses load-then-store — concurrent transitions can lose state

## Invariant
`changeStateTo()` must produce a valid, intended state even under concurrent calls on the same object.

## Code Locus
`LifeCycle.hpp:182–200` — `StateMachine::changeStateTo()`.

```cpp
oldState = _state.load(acquire);            // step 1: snapshot
if (!isValidTransition(oldState, newState)) // step 2: validate snapshot
    return error;
setAndNotifyState(newState);                // step 3: unconditional store (NOT CAS)
```

## Possible Violation
Between step 1 (load) and step 3 (store), another thread can change `_state`. The store at step 3 unconditionally overwrites the current state.

### Concrete race scenario
- **Pool worker** detects ERROR from `work()` and calls `scheduler.changeStateTo(ERROR)` (Scheduler.hpp:660)
- **External thread** calls `scheduler.changeStateTo(REQUESTED_STOP)` to stop the scheduler
- Both load `RUNNING`, both validate as valid, last writer wins
- If `REQUESTED_STOP` overwrites `ERROR`: the scheduler stops "normally" and `runAndWait()` returns success even though an error occurred

### Analysis of actual concurrent callers

| Thread A | Thread B | Risk |
|----------|----------|------|
| poolWorker → `ERROR` | External → `REQUESTED_STOP` | **ERROR lost** — scheduler appears to stop normally |
| poolWorker → `ERROR` | poolWorker → `ERROR` | Benign — same target state |

For **block** state (as opposed to scheduler state), the TOCTOU is mitigated by finding 007: `stop()` calls `waitDone()` before transitioning blocks, ensuring workers have exited. So block-level concurrent `changeStateTo` calls don't occur in practice.

## Enforcement
Documentation-only. The TOCTOU is noted in a contract comment above `changeStateTo()`.

A CAS loop would be the correct fix but requires architectural changes:
- The lifecycle callbacks (lines 206–234) use `oldState` to select which callback to fire
- A CAS loop that retries on failure would need to re-evaluate callbacks, which may have side effects
- The callback invocation and state store are not atomic with respect to each other

## Why Documentation Is Appropriate Here
- The race window is narrow (load-to-store in the same function)
- The dangerous combination (ERROR vs REQUESTED_STOP) requires an external stop request arriving at the exact moment a worker detects an error
- In the ERROR-lost case, the scheduler still stops cleanly — the symptom is a missing error report, not corruption or crash
- A CAS-based fix would be a significant architectural change affecting every state transition
