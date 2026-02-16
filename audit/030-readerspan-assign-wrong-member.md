# Finding 030: ReaderSpan::operator= references non-existent `_rangesCounter` — latent compilation error + instance count leak

## Invariant
Copy assignment must maintain the same reference-counting invariants as the copy constructor.

## Code Locus
`CircularBuffer.hpp:567–574` — `ReaderSpan::operator=`.

## Possible Violation
```cpp
ReaderSpan& operator=(const ReaderSpan& other) {
    if (this != &other) {
        _parent       = other._parent;
        _internalSpan = other._internalSpan;
        _parent->_rangesCounter++;     // ← _rangesCounter does not exist on Reader
    }
    return *this;
}
```

Two bugs:
1. **Wrong member name:** `_rangesCounter` does not exist on `Reader`. The `Reader` class (line 676) has `_instanceCount`. This is a latent compilation error — it compiles only because `ReaderSpan::operator=` is a member of a class template that is never instantiated (no code path uses copy assignment on `ReaderSpan`).

2. **Missing old-parent decrement:** When reassigning a span from one parent to another, the old parent's `_instanceCount` must be decremented before the new parent's is incremented. The copy constructor (line 565) correctly increments the new parent, and the destructor (line 577) correctly decrements. But the assignment operator fails to decrement the old parent, leaking a reference count. This would prevent the old parent's destructor cleanup from firing (the destructor only runs cleanup when `_instanceCount == 0`).

### Trigger path (if operator= were instantiated)
1. Create two `ReaderSpan` objects from different `Reader` parents
2. Assign one to the other: `span2 = span1`
3. Old parent's `_instanceCount` is not decremented → destructor cleanup skipped
4. New parent's count incremented via non-existent member → compilation error

## Fix
Replace `_rangesCounter` with `_instanceCount` and add the missing decrement:
```cpp
ReaderSpan& operator=(const ReaderSpan& other) {
    if (this != &other) {
        _parent->_instanceCount--;     // release old parent
        _parent       = other._parent;
        _internalSpan = other._internalSpan;
        _parent->_instanceCount++;     // acquire new parent
    }
    return *this;
}
```

## CI Impact
All tests pass: qa_Scheduler, qa_Graph, qa_Block, qa_LifeCycle.
