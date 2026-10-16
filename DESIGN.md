# GNU Radio 4 superbuild design

This document records the architectural choices behind the GNU Radio 4
workspace. Operational instructions belong in [the documentation index](README.md#documentation).

## Architecture

The workspace uses a CMake superbuild based on `ExternalProject`, not a unified
`add_subdirectory()` build. The high-level repository relationships are shown
in the [README diagram](README.md#repository-dependencies).

The manifest's build ordering is more specific than that conceptual view:

- Blocks consumes the installed Core package and builds the algorithm library
  in its `library/` directory.
- Incubator and Studio blocks build after Blocks.
- Control Plane compiles against Core. The superbuild schedules it after
  Blocks so the normal runtime plugin catalog is available.
- The Node Studio application shares its checkout with Studio blocks. When
  Control Plane is also selected, Studio builds after it and communicates with
  it through HTTP and WebSocket APIs rather than linking to block libraries.

Every selected CMake component installs into the active profile's shared
prefix, such as `install/` for `dev` or `install/full/` for `full`.

This retains the package boundaries developers and CI encounter when each
repository is built alone. It also lets each repository retain its own cache,
options, generated files, and test tree.

## Deliberate simplifications

- An ordered `modules.cmake` manifest declares repositories, dependencies,
  adapter types, tests, and profile groups without requiring a YAML parser.
- The same registration API is available to the ignored
  `projects.local.cmake` manifest for local and out-of-tree modules.
- CMake clones only missing first-party sources. A guarded download script
  requires the module's expected source marker, refuses to replace a malformed
  or non-empty directory, and never updates an existing working copy, so normal
  branch-based development remains safe.
- Presets select named module groups and replace shared argument files for the
  common configurations.
- Global and per-component cache variables plus `projects.local.cmake` replace
  per-repository argument files.
- Package and feature selection remains owned by each component. The superbuild
  forwards component-specific CMake arguments without mirroring every child
  option as a top-level option.
- A generated activation script contains only paths needed after installation.
- CMake source subdirectories and a small Node-project adapter cover Studio
  without turning the entire workspace into one CMake target namespace.
- A Node module may name a `SOURCE_PROVIDER` when its application and CMake
  module share one repository. This makes source ownership explicit without
  creating competing clone steps.
- The Node adapter hashes `package.json` and `package-lock.json`, running
  `npm ci` only when those inputs or `node_modules` change.
- One-off `GR4_EXTRA_PROJECTS` entries build by default but do not register
  tests. Local modules join `check` only when declared with `TESTS`.
- Scaffolding, profiling, containers, and service management remain independent
  of the build graph.

## Tradeoffs

The manifest is deliberately ordered rather than topologically sorted.
Dependencies must be declared before consumers; configuration validates this
and reports forward dependencies. The resulting file remains readable as the
actual build sequence and avoids a separate graph implementation.

`ExternalProject` does not model individual source files in the outer build.
`BUILD_ALWAYS` therefore asks each inner CMake build to run on every outer
build. With Ninja this is still incremental: unchanged repositories quickly
report that there is no work.

The outer build's configure, build, and install steps use the terminal pool, so
only one component performs one of those steps at a time. Outer build-preset
`jobs` values and `cmake --build --parallel`/`-j` are not forwarded to the
nested build tools. Nested CMake concurrency is instead controlled by
`GR4_BUILD_PARALLEL_LEVEL`, falling back to the
`CMAKE_BUILD_PARALLEL_LEVEL` environment variable. The Node adapter is
serialized with the other outer component steps, but npm, TypeScript, and Vite
retain control of their own internal concurrency.

The shared prefix is intentionally mutable. Removing or renaming installed
files can leave stale artifacts there. During normal development, remove the
`install/` directory when an install-layout change makes the prefix suspect.

Tests remain owned by each child project. The superbuild's `check` target runs
registered CTest trees, Studio's Node tests, and an installed-SDK smoke test:

```sh
cmake --build --preset full --target check
```
