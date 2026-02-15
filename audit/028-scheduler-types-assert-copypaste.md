# Finding 028: propertyCallbackRegistrySchedulerTypes asserts wrong property name — copy-paste bug

## Invariant
Debug asserts must validate the correct condition.

## Code Locus
`Graph.cpp:65` — `propertyCallbackRegistrySchedulerTypes()`.

## Possible Violation
```cpp
std::optional<Message> Graph::propertyCallbackRegistrySchedulerTypes(...) {
    assert(propertyName == graph::property::kRegistryBlockTypes);  // ← wrong constant
```

The callback is registered for `kRegistrySchedulerTypes` (Graph.cpp:12):
```cpp
propertyCallbacks[graph::property::kRegistrySchedulerTypes] = std::mem_fn(&Graph::propertyCallbackRegistrySchedulerTypes);
```

But the assert checks `kRegistryBlockTypes` — copied from the adjacent `propertyCallbackRegistryBlockTypes` function. In debug builds, this assert fires every time a scheduler-type registry query is processed, since `propertyName` will be `"RegistrySchedulerTypes"` not `"RegistryBlockTypes"`.

In release builds the assert is compiled out, so the function works correctly — the return value is `availableSchedulers()` regardless.

## Fix
Change the assert to check the correct constant:
```cpp
assert(propertyName == graph::property::kRegistrySchedulerTypes);
```

## CI Impact
All tests pass. No behavioral change in release builds.
