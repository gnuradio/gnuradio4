# Finding 023: ~Block() IO thread race — 10ms sleep instead of proper synchronization

## Invariant
`~Block()` must not complete while the blocking IO thread is still accessing `this`.

## Code Locus
`Block.hpp:849–864` — `~Block()` destructor.
`Block.hpp:2302–2303` — IO thread exit sequence.

## Possible Violation
The blocking IO thread (line 2286) captures `this` as a raw pointer. Its exit sequence is:
```cpp
emitErrorMessageIfAny("-> STOPPED", this->changeStateTo(STOPPED));  // line 2302: state becomes STOPPED
ioThreadRunning.store(false);                                        // line 2303: still using `this`
```

Meanwhile, `cleanupZombieBlocks` (line 975-977) checks block state: if STOPPED → `shouldDelete = true` → erases all shared_ptrs → `~Block()` runs.

The destructor previously used a 10ms sleep as a mitigation:
```cpp
if constexpr (blockingIO) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // NOT a guarantee
}
```

If the IO thread hasn't reached line 2303 within 10ms (e.g., thread preempted, slow system), the destructor completes and frees the block's memory. The IO thread then accesses `this` via `ioThreadRunning.store(false)` — use-after-free.

### Race Timeline
1. IO thread: `changeStateTo(STOPPED)` — state becomes STOPPED
2. Pool worker: `cleanupZombieBlocks` sees STOPPED, erases shared_ptrs
3. `~Block()` runs: 10ms sleep, then exits
4. Block memory freed
5. IO thread: `ioThreadRunning.store(false)` — **use-after-free**

## Fix
Replace the 10ms sleep with a spin-wait on `ioThreadRunning`:

```cpp
if constexpr (blockingIO) {
    while (ioThreadRunning.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
}
```

`ioThreadRunning` is set to `false` as the IO thread's last access to `this` (line 2303). Once it reads `false`, the destructor can safely proceed — the IO thread won't touch `this` again.

## Why Safe
- `ioThreadRunning` is initialized to `false` (line 710) — if the IO thread was never started, the wait exits immediately
- If the IO thread already finished, `ioThreadRunning` is already `false` — no delay
- The IO thread sets `ioThreadRunning = false` AFTER all other `this` accesses
- `std::this_thread::yield()` avoids busy-spin CPU waste while still being responsive

## CI Impact
All tests pass: qa_Scheduler (29 tests), qa_Block (all suites including BlockingIO Tests), qa_LifeCycle (8 tests).
