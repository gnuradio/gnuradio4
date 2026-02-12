# Audit Finding 001: poolWorker Stale `activeState` Cache

**Area:** Scheduler `poolWorker` loop responsiveness to lifecycle state transitions
**File:** `core/include/gnuradio-4.0/Scheduler.hpp`, lines 564–654
**Severity:** Low–Medium (correctness/determinism, not crash or UB)
**Risk of Fix:** Low

---

## Current Behavior

The `poolWorker` main loop caches the scheduler's lifecycle state in a local
variable `activeState` (line 587). This variable controls whether the loop
calls `traverseBlockListOnce` (RUNNING), sleeps (PAUSED), or exits
(`!isActive`).

`activeState` is only refreshed at line 608, inside the
`if (hasMessagesToProcess)` branch, which executes once every
`process_stream_to_message_ratio` iterations (default: **16**). On the other
15 iterations, the loop operates on a stale cached value.

```cpp
// Scheduler.hpp:587
auto activeState = this->state();     // initial read
do {
    bool hasMessagesToProcess = msgToCount == 0UZ;   // true every 16th iteration
    if (hasMessagesToProcess) {
        // ... message processing, zombie cleanup, block adoption ...
        activeState = this->state();  // ← ONLY refresh point (line 608)
        msgToCount++;
    } else {
        // ... increment msgToCount mod 16 ...
    }

    if (activeState == RUNNING) {
        traverseBlockListOnce(localBlockList);  // calls block->work() on every block
    } else if (activeState == PAUSED) {
        sleep(timeout_ms); msgToCount = 0UZ;    // resets counter, so next iteration refreshes
    } // ...
} while (lifecycle::isActive(activeState));
```

`this->state()` (defined in `LifeCycle.hpp:241`) is a single
`_state.load(std::memory_order_acquire)` — a plain `mov` on x86.

## Issue

When an external thread requests stop or pause, the pool worker continues
calling `traverseBlockListOnce` for up to **15 additional iterations** before
it notices the state change.

### Consequences

1. **Delayed pause response.** `Block::workInternal()` (`Block.hpp:2007`) does
   not check for `REQUESTED_PAUSE` or `PAUSED` — it only checks
   `REQUESTED_STOP` and `STOPPED`. So blocks continue processing data during
   those 15 extra traversals, violating the intended pause semantic.

2. **Unnecessary work on stop.** Blocks already transitioned to `STOPPED` (by
   the scheduler's `stop()`, lines 694–716) return `{0, DONE}` from `work()` —
   cheap but pointless per-block calls across the entire local job list,
   repeated up to 15 times.

3. **Coupling between unrelated parameters.** `process_stream_to_message_ratio`
   is documented as controlling message processing frequency, but it silently
   also controls state-transition response latency. Tuning the ratio to reduce
   message overhead inadvertently increases stop/pause latency.

The staleness is specific to the **RUNNING** state (the hot path). In PAUSED
and other non-running states, `msgToCount` is reset to 0, so the next
iteration refreshes immediately. Only RUNNING → {REQUESTED_STOP, PAUSED, ERROR}
transitions are affected.

## Proposed Fix

Move the `activeState` refresh out of the message-processing guard so it
runs unconditionally, between message handling and the work-dispatch decision:

```diff
     if (hasMessagesToProcess) {
         if (runnerID == 0UZ || nRunningJobs->value() == 0UZ) {
             this->processScheduledMessages();
         }

         cleanupZombieBlocks(localBlockList);
         adoptBlocks(runnerID, localBlockList);

         std::ranges::for_each(localBlockList, &BlockModel::processScheduledMessages);
-        activeState = this->state();
         msgToCount++;
     } else {
         if (std::has_single_bit(process_stream_to_message_ratio.value)) {
             msgToCount = (msgToCount + 1U) & (process_stream_to_message_ratio.value - 1);
         } else {
             msgToCount = (msgToCount + 1U) % process_stream_to_message_ratio.value;
         }
     }

+    activeState = this->state();
+
     if (activeState == RUNNING) {
```

One line moved. No new code, no new abstractions, no behavioral change in
normal streaming operation.

## Why This Helps

- **Correctness:** Stop and pause transitions are observed within one work
  cycle instead of up to 16. Pause semantics become deterministic.
- **Clarity:** Decouples message-processing frequency from state-transition
  responsiveness, making `process_stream_to_message_ratio` behave as documented.
- **Performance cost:** One additional `std::atomic::load(memory_order_acquire)`
  per iteration — a single `mov` on x86, entirely dominated by `block->work()`
  calls. Unmeasurable.

## Risk Assessment

**Low.** The change moves an existing atomic load from conditional to
unconditional execution. No new synchronization, no new code paths, no altered
data flow. The loop body already accesses `_state` indirectly through
`block->work()` on every iteration, so cache-line behavior is unchanged.

## Related Observations (not part of this fix)

- `Block::workInternal()` does not check for `REQUESTED_PAUSE`/`PAUSED` state.
  Even with this fix, individual blocks will still execute `processBulk`/
  `processOne` when paused — the scheduler loop just won't call them. This is
  a separate concern worth a future audit step.
- The `changeStateTo` function in `LifeCycle.hpp` uses load-then-store rather
  than compare-and-swap, creating a TOCTOU window under concurrent calls. This
  is a deeper architectural question for a separate finding.
