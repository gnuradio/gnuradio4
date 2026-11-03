# Building GNU Radio 4

This guide covers the local development workspace. For the browser and desktop
Studio launch paths, see [Studio](studio.md).

## Requirements

The core stack uses C++23 and requires:

- CMake 3.27 or newer
- Ninja
- Git
- GCC 14 or newer (GCC 15+ recommended), or a current Clang (Clang 20+
  recommended)

The full workspace additionally requires Node.js 22 and npm. Individual
modules can require further system packages. Third-party dependencies are built
from source by default, as described below; the `offline` profile uses only
system dependencies.

## Third-party dependencies

The modules do not build their own dependencies. A second superbuild,
[gr-deps](https://github.com/gnuradio/gnuradio-deps), builds
vir-simd, boost.ut, cpp-httplib, SoapySDR, Boost, zlib and the rest into the
prefix before any module configures, so that one toolchain and, on Windows, one
MSVC runtime are used throughout. It is driven from here as the `gr-deps`
project: missing sources are cloned into `src/gr-deps` on the first build, and
an existing checkout is left alone exactly as a module's is.

Which dependencies are built follows the module selection. Each module declares
the gr-deps groups it consumes with `DEPS_GROUPS` in `modules.cmake`, and the
union over the selected modules decides what gr-deps is asked for, so a `base`
workspace never builds the control plane's Boost. Changing the selection
rebuilds only what the new groups add.

| Variable | Default | Effect |
| --- | --- | --- |
| `GR4_BUILD_DEPS` | `ON` | `OFF` uses dependencies already on the machine and creates no `gr-deps` project |
| `GR4_DEPS_INSTALL_PREFIX` | `CMAKE_INSTALL_PREFIX` | Where the dependencies install; a separate prefix is added to every module's search path |
| `GR4_DEPS_SOURCE_DIR` | `src/gr-deps` | An existing gr-deps checkout to build instead |
| `GR4_DEPS_REPOSITORY`, `GR4_DEPS_REF` | the gr-deps repository above, `main` | What is cloned when no checkout exists |
| `GR4_DEPS_BUILD_TYPE` | follows `CMAKE_BUILD_TYPE` | Build type for the dependencies; a build type they do not know, such as `RelWithAssert`, becomes `Release` |
| `GR4_DEPS_CMAKE_ARGS` | empty | Extra arguments for gr-deps itself, such as `-DENABLE_TBB=ON` |

`GR4_BUILD_TESTING` reaches the dependencies as well, as gr-deps'
`TARGET_TESTS`: a build with tests off does not build boost.ut or GoogleTest,
which nothing but the child test suites uses. The install layout and the
toolchain are forwarded to each dependency build too, so a dependency lands in
the same `lib/` and `bin/` as the modules rather than wherever `GNUInstallDirs`
would put it on the distribution in use.

Build or discard them on their own:

```sh
cmake --build --preset dev --target gr-deps
cmake --build --preset dev --target gr-deps-clean
```

A prefix built elsewhere, such as one restored from the gr-deps CI cache, is
used by configuring with `-DGR4_BUILD_DEPS=OFF
-DGR4_DEPS_INSTALL_PREFIX=/path/to/prefix`: nothing is built, and the prefix is
still searched by every module.

## Standard profiles

Configure, build, and activate using the same profile name:

```sh
cmake --preset dev
cmake --build --preset dev
source build/dev/activate.sh
```

| Preset | Configuration | Intended use | Install prefix |
| --- | --- | --- | --- |
| `dev` | RelWithAssert, tests, warnings as errors | Normal development | `install/` |
| `debug` | Debug, tests | Debugger-friendly development | `install/debug/` |
| `release` | Release, no tests | Performance and install validation | `install/release/` |
| `asan` | Debug, tests, AddressSanitizer | Memory-error diagnostics | `install/asan/` |
| `ubsan` | Debug, tests, UndefinedBehaviorSanitizer | Undefined-behavior diagnostics | `install/ubsan/` |
| `offline` | RelWithAssert, tests, system dependencies | Dependencies already on the machine, none built | `install/offline/` |
| `ci` | RelWithAssert, tests, warnings as errors | Base-stack integration CI | `install/ci/` |
| `full` | RelWithAssert, tests, incubator, control-plane, Studio | Complete application development | `install/full/` |
| `full-ci` | RelWithAssert, tests, incubator, control-plane, Studio | Complete integration CI | `install/full-ci/` |

Discover the profiles available in the current checkout:

```sh
cmake --list-presets
cmake --build --list-presets
```

## Control parallelism

Set `CMAKE_BUILD_PARALLEL_LEVEL` before configuring if compilation uses too
much memory. It is propagated to nested CMake component builds:

```sh
export CMAKE_BUILD_PARALLEL_LEVEL=4
cmake --preset dev
cmake --build --preset dev
```

To store the limit in a profile instead, pass
`-DGR4_BUILD_PARALLEL_LEVEL=4` during configuration. That option takes
precedence over `CMAKE_BUILD_PARALLEL_LEVEL`.

`cmake --build --parallel N`, `-jN`, and a build preset's `jobs` value control
only the outer superbuild; they do not limit the nested component builds. The
Studio npm, TypeScript, and Vite steps use their own concurrency controls.

## Source ownership and incremental builds

On the first build, CMake clones missing component repositories into `src/` at
their configured refs. Once a checkout exists, the superbuild does not fetch,
pull, switch, reset, or replace it. Updating it and choosing its branch are the
developer's responsibility. A newly cloned source may be detached; create a
branch in the child repository before making commits.

The child builds are separate and share one install prefix. Re-running the
outer build invokes required child builds, while their Ninja builds remain
incremental. To clone selected sources without compiling:

```sh
cmake --build --preset dev --target sources
```

Use `full` instead of `dev` to include incubator, control-plane, and Studio.
Remove one child's build state without deleting its source or shared prefix
(`gr-deps-clean` does the same for the dependency superbuild, leaving the
dependencies it already installed in place):

```sh
cmake --build --preset dev --target gnuradio4-blocks-clean
```

## Build individual components

Targets include their upstream dependencies:

```sh
cmake --build --preset dev --target gnuradio4-core
cmake --build --preset dev --target gnuradio4-library
cmake --build --preset dev --target gnuradio4-blocks
```

For full-workspace targets:

```sh
cmake --build --preset full --target gnuradio4-control-plane
cmake --build --preset full --target gr4-incubator
cmake --build --preset full --target gnuradio4-studio-blocks
cmake --build --preset full --target gnuradio4-studio
```

## macOS with Homebrew

The Apple Clang supplied by the Command Line Tools can be too old for the
C++23 features used by GR4. `CMakeUserPresets.json.mac.example` selects
Homebrew LLVM and provides `dev-mac` and `full-mac` profiles. It assumes the
Apple Silicon Homebrew prefix; replace `/opt/homebrew` with `/usr/local` on
Intel Macs.

```sh
brew install cmake ninja llvm
cp CMakeUserPresets.json.mac.example CMakeUserPresets.json

cmake --preset dev-mac
cmake --build --preset dev-mac
source build/dev-mac/activate.sh
```

Use `full-mac` for the complete workspace. It also needs Node.js 22 and npm.
If ccache cannot write to its cache directory:

```sh
CCACHE_DISABLE=1 cmake --build --preset full-mac
```

## Machine-local profiles

Keep local profiles in ignored `CMakeUserPresets.json`:

```json
{
  "version": 6,
  "configurePresets": [
    {
      "name": "my-radio",
      "inherits": "dev",
      "binaryDir": "${sourceDir}/build/my-radio",
      "cacheVariables": {
        "CMAKE_INSTALL_PREFIX": "${sourceDir}/install/my-radio",
        "GR4_BUILD_PARALLEL_LEVEL": "4",
        "GR4_MODULE_GROUPS": "base;experimental",
        "GR4_BLOCKS_CMAKE_ARGS": "-DENABLE_EXAMPLES=OFF"
      }
    }
  ],
  "buildPresets": [
    {
      "name": "my-radio",
      "configurePreset": "my-radio",
      "jobs": 0
    }
  ]
}
```

Then use it like a supplied profile:

```sh
cmake --preset my-radio
cmake --build --preset my-radio
source build/my-radio/activate.sh
```

See [Module configuration](modules.md) for module selection and the complete
top-level option reference.
