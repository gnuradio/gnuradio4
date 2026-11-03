# Extending the workspace

## Work on component repositories

Repositories under `src/` are ordinary Git working copies. Sources cloned by
the superbuild start at a detached commit for the requested ref, so create a
branch before committing:

```sh
cd src/gnuradio4-blocks
git switch -c my-feature
```

Commit and push in the owning repository; use that project's own development
documentation and issue tracker. Reconfiguring this workspace does not fetch,
update, switch, or reset child working copies.

Use checkouts stored elsewhere with:

```sh
cmake --preset dev -DGR4_SOURCE_ROOT=/path/to/sources
cmake --build --preset dev
```

The selected repositories that are absent from the source root are cloned into
it. `activate.sh` adds the active prefix to executable, CMake package,
pkg-config, runtime-library, Python site-package, and GR4 plugin search paths.

## Add an out-of-tree project

Build an external CMake project in the same superbuild:

```sh
cmake --preset dev -DGR4_EXTRA_PROJECTS=/path/to/gr4-example
cmake --build --preset dev --target gr4-example
```

Extra projects depend on the core/library/blocks stack, and on the third-party
dependencies that stack needs, and are built by the aggregate target. The `GR4_EXTRA_PROJECTS` shortcut does not register their
tests. For test integration or persistent dependency relationships, register a
module in `projects.local.cmake` and include the `TESTS` keyword.

For persistent local projects or relationships, copy
`projects.local.cmake.example` to ignored `projects.local.cmake` and register a
module:

```cmake
gr4_register_module(
  NAME my-gr4-module
  TYPE CMAKE
  SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../my-gr4-module"
  DEPS_GROUPS core blocks
  DEPENDS gnuradio4-blocks
  CMAKE_ARGS
    "-DENABLE_PLUGINS:BOOL=ON"
  TESTS)
```

Select it with `-DGR4_MODULES=my-gr4-module`, or give it a `GROUPS` value.
`DEPS_GROUPS` adds the third-party dependencies the module includes itself to
what is built into the prefix. See
[Module configuration](modules.md) for the registration API.
