# Finding 034: addSequences() double-initializes sequences — racy overwrite after CAS success

## Invariant
Sequence values should be initialized exactly once, atomically with their addition to the shared vector.

## Code Locus
`Sequence.hpp:95–125` — `addSequences()`.

## Possible Violation
```cpp
do {
    currentSequences = std::atomic_load_explicit(&sequences, std::memory_order_acquire);
    updatedSequences = std::make_shared<...>(...);
    std::ranges::copy(currentSequences, updatedSequences);

    cursorSequence = cursor.value();                       // line 110: read cursor
    for (auto&& sequence : sequencesToAdd) {
        sequence->setValue(cursorSequence);                 // line 114: FIRST init (inside CAS loop)
        (*updatedSequences)[index] = sequence;
    }
} while (!std::atomic_compare_exchange_weak(&sequences, &currentSequences, updatedSequences));

cursorSequence = cursor.value();                           // line 120: re-read cursor (OUTSIDE loop)
for (auto&& sequence : sequencesToAdd) {
    sequence->setValue(cursorSequence);                     // line 123: SECOND init (racy overwrite)
}
```

After the CAS succeeds at line 118, the new sequences are atomically visible in the shared vector. Other threads (e.g., `getMinReaderCursor()` in the writer's `getRemainingCapacity()`) can immediately read the sequence values set at line 114.

Lines 120–124 then re-read the cursor and overwrite the sequence values. Between the CAS success and line 123, the writer may have:
1. Read `getMinReaderCursor()` using the value from line 114
2. Reserved buffer space based on that value
3. Line 123 overwrites to a newer (higher) cursor value

While this particular race is benign (newer cursor = more conservative reader position), the code is logically wrong:
- The sequence values should reflect the cursor at the time they were atomically added
- The re-init at line 123 uses a different cursor value from a different point in time
- The `xTODO: explicit memory order` comment at line 118 suggests this area was known to need review

## Fix
Remove the redundant re-initialization outside the CAS loop (lines 120–124). The values set inside the successful CAS iteration (line 114) are the correct ones.

## CI Impact
All tests pass: qa_Settings, qa_Scheduler, qa_Graph, qa_Block, qa_LifeCycle.
