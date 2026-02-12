# Audit Finding 008: Scheduler Messages Orphaned When Runner 0 Exits Early

**Area:** Scheduler-level message processing in `poolWorker`
**File:** `core/include/gnuradio-4.0/Scheduler.hpp`, line 597
**Severity:** Medium (scheduler stops responding to control messages while graph
is still running)
**Risk of Fix:** Low

---

## Current Behavior

Inside the `poolWorker` main loop (line 595–609), scheduler-level messages are
processed only by runner 0:

```cpp
bool hasMessagesToProcess = msgToCount == 0UZ;
if (hasMessagesToProcess) {
    if (runnerID == 0UZ || nRunningJobs->value() == 0UZ) {   // ← line 597
        this->processScheduledMessages();
    }

    cleanupZombieBlocks(localBlockList);
    adoptBlocks(runnerID, localBlockList);

    std::ranges::for_each(localBlockList, &BlockModel::processScheduledMessages);
    // ...
}
```

The fallback condition `nRunningJobs->value() == 0UZ` can never be true inside
a running pool worker (the caller itself contributes at least 1 to the count),
so in practice only `runnerID == 0UZ` fires.

## Issue

In a multi-threaded scheduler, blocks are distributed across runners via
round-robin (Simple) or graph traversal (BFS/DFS). When all of runner 0's
blocks finish (all return `work::Status::DONE`), `traverseBlockListOnce`
returns DONE, and runner 0 breaks out of the loop (line 620–621):

```cpp
if (result.status == work::Status::DONE) {
    break; // nothing happened -> shutdown this worker
}
```

Runner 0 exits and decrements `_nRunningJobs`. Meanwhile, runners 1..N are
still actively processing their blocks.

From this point:

1. No runner satisfies `runnerID == 0UZ` (runner 0 has exited)
2. No runner satisfies `nRunningJobs->value() == 0UZ` (runners 1..N are alive)
3. `processScheduledMessages()` is never called
4. **All scheduler-level messages are orphaned**

Messages affected include: `kEmplaceBlock`, `kRemoveBlock`, `kReplaceBlock`,
`kEmplaceEdge`, `kRemoveEdge`, `kGraphGRC`, `kBlockReplaced`, and any other
message processed through the scheduler's message ports. These messages
accumulate in the port's ring buffer until all remaining runners also exit.

Block-level `processScheduledMessages` (line 607) is still called by all
runners for their own blocks, so block settings and heartbeats continue.
Only the scheduler's own message queue is orphaned.

### When Does This Happen?

- **File source exhaustion:** Source blocks (which are often among the first
  assigned to runner 0 in round-robin distribution) signal DONE when they
  reach EOF. Downstream blocks on other runners continue draining buffered
  data.
- **Conditional stop:** A source block may signal DONE based on sample count
  or external trigger, while downstream blocks continue processing.
- **Uneven graph partitioning:** If runner 0 receives fewer or lighter blocks,
  it finishes first.

The window is not instantaneous — it persists from runner 0's exit until all
remaining runners also exit, which could be significant for deep pipelines.

## Proposed Fix

Remove the runner-0 guard. The `processScheduledMessages()` method already has
an internal atomic flag (`_processingScheduledMessages`, line 341) that
prevents concurrent execution:

```cpp
void processScheduledMessages() {
    if (std::atomic_flag_test_and_set_explicit(&_processingScheduledMessages, std::memory_order_acquire)) {
        return; // another thread is already processing
    }
    on_scope_exit _ = [&] { std::atomic_flag_clear_explicit(&_processingScheduledMessages, ...); };
    // ... process messages ...
}
```

The fix:

```diff
-                if (runnerID == 0UZ || nRunningJobs->value() == 0UZ) {
-                    this->processScheduledMessages();
-                }
+                this->processScheduledMessages();
```

After this change, every runner calls `processScheduledMessages()` during its
message-processing cycle. The atomic flag ensures only one runner processes at
a time — all others hit the `test_and_set`, return immediately, and continue
with `cleanupZombieBlocks` / `adoptBlocks`. When runner 0 exits, any surviving
runner picks up scheduler-level message processing on its next cycle.

## Why This Helps

- **Correctness:** Scheduler-level messages are processed as long as any runner
  is alive, not just while runner 0 is alive.
- **Responsiveness:** Dynamic graph modifications (`kEmplaceBlock`,
  `kRemoveBlock`, etc.) continue to work during the tail end of graph
  execution.
- **Simplicity:** Removes a per-runner special case. The atomic flag inside
  `processScheduledMessages()` was already designed to handle multi-caller
  access.

## Risk Assessment

**Low.** One line removed, two lines deleted. The `processScheduledMessages()`
method is already designed for concurrent callers — the atomic flag at line 341
serializes access. The only behavioral change is that non-zero runners now
attempt to process scheduler messages (hitting the atomic flag and returning
immediately if another runner is already processing). The overhead per runner
is one `atomic_flag_test_and_set` per message-processing cycle — negligible
compared to `traverseBlockListOnce` work.
