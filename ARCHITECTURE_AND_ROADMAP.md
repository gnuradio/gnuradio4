# GNU Radio 4.0 — Architectural Overview & Development Roadmap

## 1. Architectural Summary

### 1.1 Core Modules

| Module | Path | Purpose |
|---|---|---|
| **core** | `core/` | Framework kernel: Block, Graph, Scheduler, Port, Buffer, Settings, PluginLoader, BlockRegistry |
| **meta** | `meta/` | Compile-time metaprogramming: reflection, typelist, formatters, UncertainValue |
| **algorithm** | `algorithm/` | DSP algorithms: FFT (SimdFFT), window functions, filter design, dataset math, SchmittTrigger |
| **blocks** | `blocks/` | Block library: basic, filter, fourier, math, electrical, fileio, http, soapy, testing |
| **blocklib_generator** | `blocklib_generator/` | Build tool: parses `GR_REGISTER_BLOCK()` macros to generate registration code |

### 1.2 Data Flow Graph Design

The architecture follows a **directed flow graph** model:

```
┌──────────────┐      ┌─────────────┐      ┌──────────────┐
│  PortOut<T>  │─────>│CircularBuffer│─────>│  PortIn<T>   │
│ (Block A)    │      │(lock-free,   │      │ (Block B)    │
│              │      │ mmap-backed) │      │              │
└──────────────┘      └─────────────┘      └──────────────┘
```

**Key design properties:**

- **Graph** (`Graph.hpp`) — Container of `BlockModel` shared pointers and `Edge` objects.
  Satisfies the `GraphLike` concept requiring `blocks()` and `edges()` accessors returning spans.

- **Block** (`Block.hpp`) — CRTP base template `Block<Derived>`. Each block declares typed ports
  (`PortIn<T>`, `PortOut<T>`) and implements `processOne()` or `processBulk()`. SIMD vectorization
  happens automatically via `simdize_tuple_load_and_apply()`.

- **Scheduler** (`Scheduler.hpp`) — Itself a `Block<Derived>` via CRTP, with three `ExecutionPolicy`
  options: `singleThreaded`, `multiThreaded`, `singleThreadedBlocking`. Uses `lifecycle::StateMachine`
  for state transitions and `JobLists` for parallel dispatch.

- **CircularBuffer** (`CircularBuffer.hpp`) — Lock-free ring buffer backed by
  `double_mapped_memory_resource` (POSIX `mmap` with double-mapping for wrap-around-free reads).
  Uses `ClaimStrategy`, `WaitStrategy`, and `Sequence` for LMAX Disruptor-style inter-block communication.

- **Tags and Messages** — Stream metadata (`Tag.hpp`) propagates in-band with samples.
  Asynchronous control uses `Message` objects routed through `msgIn`/`msgOut` ports.

### 1.3 Key Dependencies

| Dependency | Mechanism | Purpose |
|---|---|---|
| **vir-simd** v0.4.4 | FetchContent | `std::experimental::simd` implementation for portable SIMD |
| **pmt** (Polymorphic Message Type) | FetchContent (pinned SHA) | Type-erased value transport between blocks |
| **Boost.UT** | FetchContent (pinned SHA) | Unit testing framework |
| **cpp-httplib** v0.18.1 | FetchContent | HTTP client/server for REST blocks |
| **SoapySDR** | `find_package` (optional) | SDR hardware abstraction |
| **magic_enum** | Vendored in `third_party/` | Enum-to-string reflection |
| **exprtk** | Vendored in `third_party/` | Runtime math expression evaluation |

### 1.4 Abstraction Layers

```
Layer 4:  Application / Flowgraph YAML    (Graph_yaml_importer, GRC files)
Layer 3:  Block Library                    (blocks/basic, filter, fourier, soapy, ...)
Layer 2:  Framework Core                   (Block, Graph, Scheduler, Settings, PluginLoader)
Layer 1:  Infrastructure                   (CircularBuffer, Port, Tag, Sequence, LifeCycle)
Layer 0:  Meta / Algorithms                (meta/reflection, typelist; algorithm/FFT, window)
```

Each layer depends only downward. Blocks at Layer 3 see only the `Block<>` template and `Port`
types from Layer 2. The scheduler orchestrates at Layer 2 but is itself a Block. Algorithms at
Layer 0 are standalone and testable without any framework dependency.

---

## 2. Prioritized Improvements

### 2.1 Modularity

**Priority 1: Decouple algorithm library from core headers**

The `algorithm/` directory is well-separated, but some headers pull in `gnuradio-4.0/meta/utils.hpp`.
Consider making the algorithm headers self-contained or depending only on a minimal `gr::meta` subset,
so they can be consumed as a standalone DSP library.

**Priority 2: Formalize block interface versioning**

The plugin ABI version (`GR_PLUGIN_CURRENT_ABI_VERSION`) is checked at load time, but the block concept
itself (`BlockLike`) has no version. As the block API evolves, consider a
`static constexpr int block_api_version` in the `Block<>` base.

**Priority 3: Extract buffer implementations into a separate target**

`CircularBuffer.hpp` depends on `Buffer.hpp`, `ClaimStrategy.hpp`, `WaitStrategy.hpp`, and `Sequence.hpp`.
These could form a `gr-buffer` library target usable independently (e.g., in non-GR real-time audio pipelines).

### 2.2 Testability

**Priority 1: Property-based / fuzz testing for buffer logic**

The lock-free circular buffer is the highest-risk component. Existing tests should be augmented with
concurrent stress tests and property-based validation.

**Priority 2: Add integration test harness for full graph execution**

Create a test utility that builds a graph, runs the scheduler for N samples, and validates output.

**Priority 3: Benchmark regression tests in CI**

The `bench/` and `bm_*.cpp` files exist but aren't gated in CI. Add a benchmark job that records
and compares against baselines with thresholds.

### 2.3 Performance

**Priority 1: SIMD coverage for FFT and filter paths**

The FFT has hand-unrolled stages for N=2,4,8 but falls back to a scalar loop for larger stages.
Use `vir::simd` explicitly for the twiddle-multiply loop. Pre-compute aligned twiddle tables for
common FFT sizes (1024, 2048, 4096).

**Priority 2: Thread affinity and NUMA awareness in scheduler**

The multi-threaded scheduler uses `TaskExecutor` but doesn't pin threads to cores.
For high-throughput SDR applications (>10 MS/s), add optional CPU affinity.

**Priority 3: Batch tag processing**

Tags are currently processed per-sample in many blocks. For high-rate streams, amortize tag checks
over chunks by pre-scanning the tag map for the current buffer window.

---

## 3. Concrete Recommendations for Expansion

### 3.1 Adding New GNU Radio Blocks

Example: AGC (Automatic Gain Control) block

```cpp
// blocks/basic/include/gnuradio-4.0/basic/AGC.hpp
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK("gr::blocks::basic::AGC", gr::blocks::basic::AGC, ([T]), [float, double])

template<typename T>
struct AGC : gr::Block<AGC<T>> {
    using Description = Doc<R""(
Automatic Gain Control block.
Adjusts signal amplitude to maintain a target output level.
)"">;

    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    Annotated<T, "target level", Unit<"V">, Visible>     reference = T(1.0);
    Annotated<T, "attack rate", Doc<"convergence rate">>  rate      = T(1e-4);
    Annotated<T, "max gain", Unit<"dB">>                  max_gain  = T(65536);

    GR_MAKE_REFLECTABLE(AGC, in, out, reference, rate, max_gain);

    T _gain = T(1.0);

    [[nodiscard]] constexpr T processOne(T input) noexcept {
        const T output = input * _gain;
        _gain += rate * (reference - std::abs(output));
        _gain = std::clamp(_gain, T(0), max_gain);
        return output;
    }
};
} // namespace gr::blocks::basic
```

### 3.2 Modern CMake and Packaging

- Add `CMakePresets.json` for reproducible developer builds (dev-gcc, dev-clang, release)
- Export proper CMake package config for downstream `find_package(gnuradio4)`
- Add CPack configuration for `.deb` / `.rpm` / `.tar.gz` packaging

### 3.3 Python Bindings

Use **nanobind** for Python bindings exposing Graph, Scheduler, and block construction:

```cmake
option(ENABLE_PYTHON "Build Python bindings" OFF)
if(ENABLE_PYTHON)
    find_package(Python 3.12 COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(nanobind CONFIG REQUIRED)
    nanobind_add_module(gr4 bindings/python/gr4_python.cpp)
    target_link_libraries(gr4 PRIVATE gnuradio4-core gnuradio4-blocks)
endif()
```

### 3.4 CI/CD Enhancements

- Add static analysis job (clang-tidy + cppcheck)
- Add coverage gating (fail if coverage drops below threshold)
- Add benchmark regression tracking

---

## 4. Roadmap: Plugin Ecosystem, GPU, SDR Frontends

### 4.1 Plugin Ecosystem

**Phase 1:** Plugin manifest and discovery system
**Phase 2:** Plugin SDK template repository
**Phase 3:** Package manager integration (`gr4pkg install`)

### 4.2 GPU Acceleration (CUDA / ROCm)

Strategy: GPU-accelerated algorithm backends, not GPU-resident blocks.

**Phase 1:** GPU FFT backend (cuFFT / rocFFT)
**Phase 2:** Dispatch mechanism (auto-detect GPU, fallback to CPU)
**Phase 3:** Batched GPU pipeline for channelizers/filterbanks

```cmake
option(GR_ENABLE_CUDA "Enable CUDA GPU acceleration" OFF)
option(GR_ENABLE_ROCM "Enable ROCm GPU acceleration" OFF)
```

### 4.3 SDR Frontend Support

The existing SoapySDR block provides hardware abstraction for RTL-SDR, LimeSDR, HackRF.

**Expansion priorities:**
- TX support (SoapyTxBlock)
- Device discovery block
- Native driver blocks for lowest latency (librtlsdr, libhackrf)

---

## 5. Real-World SDR Testing Strategies

### 5.1 Layered Test Approach

```
Level 4: System/Integration (real hardware)     — Weekly, manual or dedicated CI runner
Level 3: Hardware-in-the-loop (loopback)         — Nightly, requires SDR hardware
Level 2: Recorded IQ file replay                 — Every CI run
Level 1: Unit tests (synthetic data)             — Every CI run
```

### 5.2 Loopback Test Procedure

1. Generate known test signal (FM-modulated 1 kHz tone at -30 dBFS)
2. TX path: `SignalGenerator -> FMModulator -> SoapyTxBlock`
3. RX path: `SoapyBlock -> FMDemodulator -> SNR_Estimator`
4. Assert: Demodulated frequency within 10 Hz, SNR > 30 dB
5. Connect TX->RX via SMA cable + attenuator (30-40 dB)

### 5.3 CI Hardware Runner

Use self-hosted runners with attached SDR hardware for nightly integration tests.

---

## 6. Step-by-Step Development Plan

| Phase | Focus | Deliverables |
|---|---|---|
| **1** | Foundation | CMakePresets.json, CPack config, cmake export, static analysis CI job |
| **2** | Block expansion | AGC, PLL, AM/FM demod, Resampler, DC blocker blocks with unit tests |
| **3** | Python bindings | nanobind-based `gr4` module, pip-installable wheel, Jupyter examples |
| **4** | Plugin SDK | Template repo, manifest system, `gr4pkg` tool, documentation |
| **5** | GPU acceleration | CUDA/ROCm FFT backends, dispatch mechanism, benchmarks |
| **6** | SDR frontends | SoapySDR TX block, native RTL-SDR block, IQ file source/sink, loopback tests |
| **7** | Performance | Thread affinity, NUMA scheduling, batch tag processing, SIMD FFT coverage |
| **8** | Ecosystem | Plugin registry, CI for third-party plugins, documentation site |
