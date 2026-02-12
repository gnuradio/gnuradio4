# Audit Finding 004: Replaced Block Never Adopted by Scheduler Workers

**Area:** Dynamic block replacement during scheduler execution
**Files:** `core/include/gnuradio-4.0/Scheduler.hpp`, lines 1089–1120 (`propertyCallbackReplaceBlock`)
**Severity:** High (new block silently never executes after replacement)
**Risk of Fix:** Low

---

## Current Behavior

When a block is dynamically replaced via the `kReplaceBlock` message, the
scheduler's `propertyCallbackReplaceBlock` handler (line 1089):

1. Calls `targetGraph->replaceBlock(...)` — which creates the new block, adds
   it to the graph's `_blocks` list, and re-points all edges to the new block
   (`Graph.cpp:29–56`)
2. Calls `makeZombie(oldBlock)` — which correctly removes the old block from
   execution via the zombie cleanup mechanism
3. Emits a `kBlockReplaced` notification with the new block's serialized data
4. Returns

```cpp
std::optional<Message> propertyCallbackReplaceBlock(...) {
    // ...
    auto [oldBlock, newBlockRaw] = targetGraph->replaceBlock(uniqueName, type, properties);
    makeZombie(std::move(oldBlock));
    // ... serialize and return notification ...
    return result;
    // ← new block is NOT added to _adoptionBlocks
    // ← new block is NOT transitioned to RUNNING
}
```

Compare with `propertyCallbackEmplaceBlock` (lines 769–796), which handles
the same situation for newly added blocks:

```cpp
std::optional<Message> propertyCallbackEmplaceBlock(...) {
    // ...
    auto& newBlock = targetGraph->emplaceBlock(type, properties);

    if (lifecycle::isActive(this->state())) {
        const auto nBatches = _adoptionBlocks.size();
        if (nBatches > 0) {
            std::lock_guard guard(_adoptionBlocksMutex);
            auto runnerIndex = ...;
            _adoptionBlocks[runnerIndex].push_back(newBlock);    // ← adopted

            // ... transition to INITIALISED then RUNNING ...     // ← started
        }
    }
    // ...
}
```

## Issue

After a `kReplaceBlock` message is processed:

1. The **old block** is correctly zombified — pool workers will stop scheduling
   it via `cleanupZombieBlocks`
2. The **new block** exists in the graph's `_blocks` list with correct edge
   references
3. But the new block is **not** in any worker's `localBlockList` and **not** in
   `_adoptionBlocks`
4. No pool worker ever calls `work()` on the new block
5. The new block remains in IDLE state — never transitioned to INITIALISED or
   RUNNING

**The replacement block is effectively dead.** It exists in the graph topology
but is never scheduled. Data flows up to the block's input ports (buffers fill)
and stops. Downstream blocks see no new data.

The caller receives a `kBlockReplaced` success notification, giving no
indication that the new block is not running.

## Proposed Fix

Add the same adoption logic from `propertyCallbackEmplaceBlock` to
`propertyCallbackReplaceBlock`:

```diff
     auto [oldBlock, newBlockRaw] = targetGraph->replaceBlock(uniqueName, type, properties);
     makeZombie(std::move(oldBlock));

+    if (lifecycle::isActive(this->state())) {
+        const auto nBatches = _adoptionBlocks.size();
+        if (nBatches > 0) {
+            std::lock_guard guard(_adoptionBlocksMutex);
+            auto blockAddress = reinterpret_cast<std::uintptr_t>(&newBlockRaw);
+            auto runnerIndex  = (blockAddress / sizeof(void*)) % nBatches;
+            _adoptionBlocks[runnerIndex].push_back(newBlockRaw);
+
+            switch (newBlockRaw->state()) {
+            case STOPPED:
+            case IDLE:
+                this->emitErrorMessageIfAny("replaceBlock -> INITIALISED", newBlockRaw->changeStateTo(INITIALISED));
+                this->emitErrorMessageIfAny("replaceBlock -> RUNNING", newBlockRaw->changeStateTo(RUNNING));
+                break;
+            case INITIALISED:
+                this->emitErrorMessageIfAny("replaceBlock -> RUNNING", newBlockRaw->changeStateTo(RUNNING));
+                break;
+            default:
+                this->emitErrorMessage("propertyCallbackReplaceBlock",
+                    std::format("Unexpected block state during replacement: {}", magic_enum::enum_name(newBlockRaw->state())));
+                break;
+            }
+        }
+    }
+
     std::optional<Message> result = gr::Message{};
```

This mirrors the adoption logic in `propertyCallbackEmplaceBlock` lines
769–796. Ideally this would be extracted into a shared helper to avoid
duplication, but that's a cleanup step and not required for correctness.

## Why This Helps

- **Correctness:** Replaced blocks are actually scheduled and process data,
  matching the user's expectation from the success notification.
- **Consistency:** `replaceBlock` and `emplaceBlock` follow the same
  adoption protocol.
- **Debuggability:** Without this fix, block replacement silently produces a
  graph that looks correct (edges connected, block present) but doesn't flow
  data — an extremely difficult failure to diagnose.

## Risk Assessment

**Low.** The fix adds code that already exists and is exercised in
`propertyCallbackEmplaceBlock`. The adoption mechanism (`_adoptionBlocks` →
`adoptBlocks` in poolWorker) is well-tested through the emplace path.
The new block is already in the graph and has valid edges; adoption
just makes the scheduler aware of it.
