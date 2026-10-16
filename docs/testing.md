# Testing

Tests remain owned by their component repositories. Run an individual child
test tree after building its profile:

```sh
ctest --test-dir build/dev/projects/gnuradio4-core --output-on-failure
ctest --test-dir build/dev/projects/gnuradio4-blocks --output-on-failure
```

The blocks test tree includes the tests of the algorithm library in
`gnuradio4-blocks` `library/`.

Run the selected workspace's complete integration check with:

```sh
cmake --build --preset dev --target check
```

`check` is an integration gate rather than an aggregate component-QA runner.
It builds an external OOT consumer against the installed SDK. In the full
profile, it also starts the installed control plane, verifies its health
endpoint, and confirms that its GNU Radio plugin-backed block catalog is
nonempty. Component unit and QA tests, including Studio lint and Vitest, remain
in their owning repositories' CI. The `release` profile disables explicitly
registered child tests; all other supplied profiles permit them.

`check` additionally builds and runs `tests/oot-smoke`, an external consumer
that discovers all three installed CMake packages. It verifies the installed
SDK rather than allowing headers or CMake targets from source trees to leak
into consumer builds.
