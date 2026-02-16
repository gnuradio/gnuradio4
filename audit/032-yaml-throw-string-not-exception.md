# Finding 032: Graph_yaml_importer throws std::string instead of std::exception — uncatchable by standard handlers

## Invariant
All thrown exceptions must derive from `std::exception` so that `catch(const std::exception&)` handlers can catch them.

## Code Locus
`Graph_yaml_importer.hpp` — 6 sites: lines 87, 124, 145, 188, 195, 205.

## Possible Violation
```cpp
throw std::format("Unable to parse exported port ({} instead of 3 elements)", ...);  // line 87
throw std::format("Unable to create scheduler of type '{}'", schedulerId);            // line 124
throw std::format("Unable to create block of type '{}'", blockType);                  // line 145
throw std::format("Unable to parse connection ({} instead of >=4 elements)", ...);    // line 188
throw std::format("Unknown block '{}'", blockName);                                   // line 195
throw std::format("Port definition has invalid length ({} instead of 2)", ...);       // line 205
```

`std::format()` returns `std::string`. Throwing a `std::string` is valid C++, but `std::string` does not derive from `std::exception`. The caller in `propertyCallbackGraphGRC` (Scheduler.hpp:1130) catches `const std::exception& e`:

```cpp
try {
    auto newGraph = gr::loadGrc(pluginLoader, yamlContent);
    // ...
} catch (const std::exception& e) {
    message.data = std::unexpected(Error{std::format("Error parsing YAML: {}", e.what())});
}
```

The `std::string` exception is NOT caught by this handler. It propagates through `processScheduledMessages()` into `poolWorker()`, which is `noexcept` → `std::terminate()`.

### Trigger path
1. User sends a `kGraphGRC` Set message with malformed YAML
2. `loadGrc()` → `loadGraphFromMap()` → encounters unknown block type
3. `throw std::format("Unable to create block of type '...'")` — throws `std::string`
4. `catch(const std::exception&)` at line 1130 does NOT catch `std::string`
5. Exception propagates to `processScheduledMessages()` → `poolWorker()` (noexcept)
6. `std::terminate()` called

### Secondary issue (also fixed)
Line 87: error message said "instead of 4 elements" but the size check was `!= 3`. Copy-paste bug from an adjacent check.

## Fix
Replace all 6 `throw std::format(...)` with `throw gr::exception(std::format(...))`:
```cpp
throw gr::exception(std::format("Unable to create block of type '{}'", blockType));
```

Also fix the error message at line 87: "4 elements" → "3 elements".

## CI Impact
All tests pass: qa_Scheduler, qa_Graph, qa_Block, qa_LifeCycle.
