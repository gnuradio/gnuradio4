# Module configuration

`modules.cmake` is the checked-in workspace manifest. It declares repositories,
dependencies, build adapters, optional child-test integration, and membership
in named module groups.
`projects.local.cmake` is its ignored, machine-local extension.

## Select modules

| Group | Modules |
| --- | --- |
| `base` | core, blocks (with the algorithm library) |
| `full` | control-plane, Studio blocks and application |
| `experimental` | incubator |
| `applications` | control-plane, Studio blocks and application |

Select groups, add a registered module, or remove an optional module:

```sh
# Core stack plus incubator
cmake --preset dev -DGR4_MODULE_GROUPS="base;experimental"

# Add one registered module and its required dependencies
cmake --preset dev -DGR4_MODULES=gr4-incubator

# Full profile plus incubator
cmake --preset full -DGR4_MODULES=gr4-incubator
```

Required dependencies are selected automatically. Excluding one is an error.
The earlier `GR4_ENABLE_INCUBATOR`, `GR4_ENABLE_CONTROL_PLANE`, and
`GR4_ENABLE_STUDIO` switches still include their module when set to `ON`; new
profiles should use `GR4_EXCLUDE_MODULES` to disable modules.

## Register a module

Add an ordered declaration after its dependencies in `modules.cmake` or in
machine-local `projects.local.cmake`:

```cmake
gr4_register_module(
  NAME gr4-radio-astronomy
  TYPE CMAKE
  SOURCE_DIR gr4-radio-astronomy
  SOURCE_KEY RADIO_ASTRONOMY
  OPTIONS_KEY RADIO_ASTRONOMY
  REPOSITORY https://github.com/example/gr4-radio-astronomy.git
  REF main
  GROUPS full
  DEPENDS gnuradio4-blocks
  TESTS)
```

The declaration creates build, download, clean, and aggregate integration and
exposes repository/ref/CMake-argument cache variables. `TESTS` enables and
registers the child test suite with the top-level `check` target. It is intended
for a locally registered project that does not have its own CI; first-party
component QA remains in its owning repository. The `CMAKE` and `NODE` module
types use generic adapters; `SOURCE_SUBDIR` covers a CMake project below a
repository root.

## Registration reference

| Argument | Meaning |
| --- | --- |
| `NAME` | Required unique target and module name |
| `TYPE` | `CMAKE` by default, or `NODE` |
| `SOURCE_DIR` | Source path; relative paths resolve below `GR4_SOURCE_ROOT` |
| `SOURCE_SUBDIR` | CMake project directory below repository root |
| `REPOSITORY` | Git URL used only when the source is missing |
| `REF` | Branch, tag, or commit; defaults to `main` |
| `SOURCE_KEY` | Cache-variable stem for `REPOSITORY` and `REF` overrides |
| `OPTIONS_KEY` | Cache-variable stem for component CMake arguments and build tree |
| `GROUPS` | Named groups that select the module |
| `DEPENDS` | Required registered modules, selected automatically |
| `OPTIONAL_DEPENDS` | Dependencies used only when independently selected |
| `SOURCE_PROVIDER` | For `NODE`, module that populates the shared repository |
| `CMAKE_ARGS` | Fixed additional arguments for a `CMAKE` child |
| `TESTS` | Enable and register child tests with the top-level `check` target |

Declarations are ordered: dependencies and source providers must come first.
Configuration rejects unknown arguments, missing values, unknown modules,
invalid cache keys, self-dependencies, forward dependencies, cycles, and
attempts to exclude a required dependency. `SOURCE_PROVIDER` and `REPOSITORY`
are mutually exclusive. Node-specific customization belongs in the owning
repository's package scripts rather than `CMAKE_ARGS`.

## Top-level options

| Option | Default | Description |
| --- | ---: | --- |
| `GR4_SOURCE_ROOT` | `src/` | First-party source repository location |
| `GR4_MODULE_GROUPS` | `base` | Registered module groups to build |
| `GR4_MODULES` | empty | Additional registered modules |
| `GR4_EXCLUDE_MODULES` | empty | Modules to remove from selected groups |
| `GR4_EXTRA_PROJECTS` | empty | Additional CMake project paths |
| `GR4_EXTRA_CMAKE_ARGS` | empty | CMake arguments for every child project |
| `GR4_BUILD_PARALLEL_LEVEL` | empty | Nested CMake job limit |
| `GR4_CORE_CMAKE_ARGS` | empty | Core-only CMake arguments |
| `GR4_BLOCKS_CMAKE_ARGS` | empty | Blocks-only CMake arguments |
| `GR4_INCUBATOR_CMAKE_ARGS` | empty | Incubator-only CMake arguments |
| `GR4_CONTROL_PLANE_CMAKE_ARGS` | empty | Control-plane-only CMake arguments |
| `GR4_STUDIO_BLOCKS_CMAKE_ARGS` | empty | Studio-blocks-only CMake arguments |
| `GR4_CORE_REPOSITORY`, `GR4_CORE_REF` | GNU Radio, `main` | Core clone source and revision |
| `GR4_BLOCKS_REPOSITORY`, `GR4_BLOCKS_REF` | GNU Radio, `main` | Blocks clone source and revision |
| `GR4_INCUBATOR_REPOSITORY`, `GR4_INCUBATOR_REF` | GNU Radio, `main` | Incubator clone source and revision |
| `GR4_CONTROL_PLANE_REPOSITORY`, `GR4_CONTROL_PLANE_REF` | GNU Radio, `main` | Control-plane clone source and revision |
| `GR4_STUDIO_REPOSITORY`, `GR4_STUDIO_REF` | GNU Radio, `main` | Studio clone source and revision |
| `GR4_CORE_SOURCE_DIR`, `GR4_BLOCKS_SOURCE_DIR`, ... | empty | Existing checkout to build instead of the one below `GR4_SOURCE_ROOT` |
| `GR4_CORE_BINARY_DIR`, `GR4_BLOCKS_BINARY_DIR`, ... | empty | Build tree to use instead of the one below the superbuild's binary directory |
| `GR4_BUILD_TESTING` | `ON` | Allow explicitly registered child-project tests |
| `GR4_FETCH_DEPS` | `ON` | Allow selected dependency downloads |
| `GR4_WARNINGS_AS_ERRORS` | `ON` | Treat child warnings as errors |
| `GR4_ADDRESS_SANITIZER` | `OFF` | Enable AddressSanitizer in children |
| `GR4_UB_SANITIZER` | `OFF` | Enable UndefinedBehaviorSanitizer in children |
| `BUILD_SHARED_LIBS` | `ON` | Build shared libraries |
| `CMAKE_INSTALL_PREFIX` | `install/` | Shared installation prefix |
| `CMAKE_INSTALL_LIBDIR` | `lib` | Library directory under the prefix |
| `GR_ENABLE_PACKAGING` | `OFF` | Build the unified installer; see the [packaging guide](packaging.md) for the `GR4_BUNDLE_*` and `GR4_MACOS_*` options it enables |

`GR4_STUDIO_REPOSITORY` and `GR4_STUDIO_REF` select the checkout shared by the
Studio-blocks and Node application modules.

`GR4_<SOURCE_KEY>_SOURCE_DIR` builds a module from an existing checkout anywhere
on disk, for example `-DGR4_CORE_SOURCE_DIR=C:/dev/gnuradio/gr-core` or the same
key under a preset's `cacheVariables`. The path is the repository root, also for
modules with a `SOURCE_SUBDIR`. It is never cloned into, so configure fails if
the module is not already there. Like the repository variables it belongs to the
checkout, so `GR4_STUDIO_SOURCE_DIR` moves both Studio modules.

`GR4_<OPTIONS_KEY>_BINARY_DIR` builds a module in a given directory instead of
`projects/<module>` below the superbuild's binary directory, for example
`-DGR4_CORE_BINARY_DIR=C:/dev/gnuradio/gr-core/build`. The directory is created
if missing, and an existing build tree is reused as it stands. A build tree
belongs to one module rather than to a checkout, so the key is the
`OPTIONS_KEY`, not the `SOURCE_KEY`: the Studio-blocks module is moved by
`GR4_STUDIO_BLOCKS_BINARY_DIR`, while the Node application module has no build
tree to move. Configure fails if two modules name the same directory, or if a
module's build tree is its own source tree. The superbuild's own bookkeeping,
the ExternalProject stamps, stays under the superbuild's binary directory, so
`<module>-clean` keeps working across a relocated build tree. Component
packaging follows the module automatically.

`GR4_EXTRA_CMAKE_ARGS` applies a child-project option to all CMake modules.
For example:

```sh
cmake --preset dev \
  -DGR4_EXTRA_CMAKE_ARGS="-DUSE_CCACHE=ON" \
  -DGR4_BUILD_PARALLEL_LEVEL=6
```

`GR_BUILD_PARALLEL_LEVEL` is a core-project option; do not pass it through
`GR4_EXTRA_CMAKE_ARGS` as a workspace-wide job limit.
