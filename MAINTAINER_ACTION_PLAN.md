# Maintainer Action Plan: Top 3 Changes for Long-Term Maintainability

**Guiding principle:** What will cause the most downstream pain if left unfixed for
another release cycle? Each change is scoped to a single PR, touches no core
abstractions, and can be reviewed independently.

---

## Change 1: Fix the Broken CMake Export

### Problem

The `gnuradio4Targets` export set is populated in three places but **never installed**:
- `core/CMakeLists.txt:73` adds `gnuradio-core`
- `meta/CMakeLists.txt:16` adds `gnuradio-meta`
- `core/src/CMakeLists.txt:21` adds `gnuradio-plugin` (separate set)

Missing:
- `install(EXPORT gnuradio4Targets ...)` — the export set is orphaned
- `gnuradio4Config.cmake.in` — does not exist
- `write_basic_package_version_file()` — no version file
- `gnuradio-algorithm` has no `install()` at all (`algorithm/CMakeLists.txt`)
- `gnuradio-blocklib-core` has no `install()` (`core/CMakeLists.txt:52`)

Result: `find_package(gnuradio4)` does not work. Every downstream consumer must
use `add_subdirectory()` or manually wire include paths.

### Files to Modify

| File | Change |
|---|---|
| `cmake/gnuradio4Config.cmake.in` | **NEW** — Package config template |
| `CMakeLists.txt` (root) | Add `install(EXPORT)`, `configure_package_config_file()`, `write_basic_package_version_file()` |
| `algorithm/CMakeLists.txt` | Add `install(TARGETS ... EXPORT gnuradio4Targets)` |
| `core/CMakeLists.txt` | Add `install()` for `gnuradio-blocklib-core` |

### Risks

- Zero ABI impact — install-tree only.
- FetchContent deps (pmt, vir) need `find_dependency()` or co-installation.
- Emscripten guard needed: `gnuradio-plugin` only exists when `NOT EMSCRIPTEN`.

### Review Criteria

1. `cmake --install build --prefix /tmp/gr4` succeeds
2. Downstream `find_package(gnuradio4 REQUIRED)` + `target_link_libraries(app PRIVATE gnuradio4::gnuradio-core)` works
3. Existing CI builds unaffected

### PR Title

```
fix(cmake): finalize install(EXPORT) and add gnuradio4Config.cmake
```

---

## Change 2: Add CTest Labels and Per-Test Timeouts

### Problem

All 60+ tests registered identically (`add_test(NAME ... COMMAND ...)`).
- No labels: cannot `ctest -L unit` vs `ctest -L integration`
- No timeouts: a hanging test blocks CI indefinitely
- No distinction between qa_AtomicBitset (ms) and qa_plugins_test (seconds)
- CI runs flat `ctest --output-on-failure` (ci.yml:158)

### Files to Modify

| File | Change |
|---|---|
| `core/test/CMakeLists.txt` | Add LABELS and TIMEOUT to `add_ut_test()` / `add_app_test()` |
| `algorithm/test/CMakeLists.txt` | Same pattern |
| `blocks/*/test/CMakeLists.txt` | Same pattern |
| `.github/workflows/ci.yml` | Split test execution by label |

### Key Changes

```cmake
# In add_ut_test():
set_tests_properties(${TEST_NAME} PROPERTIES LABELS "unit" TIMEOUT 30)

# In add_app_test():
set_tests_properties(${TEST_NAME} PROPERTIES LABELS "integration" TIMEOUT 120)

# Override for known-slow tests:
set_tests_properties(qa_Scheduler PROPERTIES LABELS "integration" TIMEOUT 120)
```

### Risks

- Zero ABI impact — build-system metadata only.
- Timeouts may be tight under ASAN; use 60s for sanitizer builds.
- Use `--no-tests=error` (CMake 3.26+) to catch label propagation bugs.

### Review Criteria

1. `ctest -L unit -N` lists only fast unit tests
2. `ctest -L integration -N` lists scheduler/plugin/graph tests
3. All existing tests still run under unfiltered `ctest`

### PR Title

```
test(cmake): add CTest labels (unit/integration) and per-test timeouts
```

---

## Change 3: Fix dlsym Undefined Behavior in Plugin Loading

### Problem

`PluginLoader.hpp:67-71` contains a self-documented FIXME:
```cpp
// FIXME: Casting a void* to function-pointer is UB in C++.
_create_fn = reinterpret_cast<plugin_create_function_t>(dlsym(_dl_handle, "gr_plugin_make"));
```

Done twice (lines 71 and 78). Undefined per C++23 [expr.reinterpret.cast]/8.
Can trap under `-fsanitize=cfi`, CHERI, and strict static analyzers.

### Solution

Export a `gr_plugin_vtable` struct from plugins. Load it via one `void*`-to-object-pointer
cast (defined behavior). Fall back to legacy symbols for old plugins.

### Files to Modify

| File | Change |
|---|---|
| `core/include/gnuradio-4.0/Plugin.hpp` | Add `gr_plugin_vtable` struct; modify `GR_PLUGIN` macro |
| `core/include/gnuradio-4.0/PluginLoader.hpp` | Replace two dlsym calls with vtable lookup + legacy fallback |

### Key Design

```cpp
// Plugin.hpp — new struct
struct gr_plugin_vtable {
    void (*make)(gr_plugin_base**);
    void (*free)(gr_plugin_base*);
};

// PluginLoader.hpp — new loading path
auto* vtbl = static_cast<gr_plugin_vtable*>(dlsym(_dl_handle, "gr_plugin_vtbl"));
if (vtbl) {
    _create_fn  = vtbl->make;    // no UB
    _destroy_fn = vtbl->free;
} else {
    // legacy fallback for old plugins (retains existing UB path)
}
```

### Risks

- ABI version stays at 1: vtable is additive (new symbol, not a change).
- Old plugins lack `gr_plugin_vtbl` — fall back gracefully.
- Verify `GNURADIO_EXPORT` covers object symbols (not just functions).

### Review Criteria

1. Existing plugin tests pass without plugin recompilation (tests legacy path)
2. After rebuild, `nm -D libplugin.so | grep gr_plugin_vtbl` shows the symbol
3. No ASAN/UBSAN/CFI warnings in plugin loading
4. FIXME at PluginLoader.hpp:67 is resolved

### PR Title

```
fix(plugins): eliminate void*-to-function-pointer UB in plugin loading
```

---

## Dependency Graph

All three changes are **independent** — they can be reviewed and merged in any
order or in parallel. None touches Block.hpp, Graph.hpp, Scheduler.hpp,
CircularBuffer.hpp, or any runtime data path.

| Priority | Change | Est. Lines | ABI Impact |
|---|---|---|---|
| 1 | CMake export fix | ~40 | None |
| 2 | CTest labels + timeouts | ~60 | None |
| 3 | Plugin dlsym UB fix | ~50 | None (additive) |
