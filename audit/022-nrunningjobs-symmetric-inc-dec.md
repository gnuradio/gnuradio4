# Finding 022: _nRunningJobs relies on symmetric inc/dec without RAII guard

## Invariant
`_nRunningJobs` must be zero when no pool workers are active.

## Code Locus
`Scheduler.hpp:607,689` — `incrementAndGet()` at entry to `poolWorker()`, `subAndGet(1)` at exit.

## Possible Violation
The increment (line 607) and decrement (line 689) are not RAII-guarded. If a pool worker exits without reaching line 689, `_nRunningJobs` remains > 0:

1. `waitDone()` (line 456) loops on `isProcessing()` which checks `_nRunningJobs->value() > 0` — hangs forever
2. The watchdog eventually transitions to ERROR, but `waitDone()` doesn't check state — only `_nRunningJobs`
3. `assert(_nRunningJobs->value() == 0)` at `start()` (line 583) catches stale counters in debug builds

### Practical Risk
After findings 008, 016, and 017, all throw paths in `poolWorker()` are contained:
- Finding 008: `processScheduledMessages()` no longer throws
- Finding 016: `tryConsume` failure logs to stderr instead of throwing
- Finding 017: User `processMessages()` wrapped in `invokeUserProvidedFunction`

The only remaining exit path is normal control flow through the `do...while(isActive(activeState))` loop, which always reaches line 689. Abnormal termination (signals, OOM) would affect the entire process regardless.

## Enforcement
Documentation-only. A contract comment is added above `_nRunningJobs` noting the symmetric inc/dec invariant and the dependency on exception containment.

## Why Documentation Is Appropriate
- An RAII guard would require a `Sequence`-aware scope guard — new abstraction for a theoretical problem
- The assert at line 583 catches violations in debug builds
- All throw paths in `poolWorker()` are now contained by other findings
- The remaining risk (signal/OOM) cannot be solved with RAII

## CI Impact
No code changes beyond documentation. All tests pass.
