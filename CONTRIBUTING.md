# Contributing to the GNU Radio 4 workspace

Thank you for contributing to GNU Radio 4. This repository is the top-level
workspace and integration superbuild: it builds independently maintained
component repositories into a single development environment. It does not own
their source code, release history, or contribution process.

All participants must follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## Choose the right repository

Submit component code changes to the repository that owns the component. The
repositories in `src/` are ordinary Git working copies; this workspace never
fetches, switches branches, or resets an existing checkout.

| Change | Submit it here? |
| --- | --- |
| Core runtime, scheduler, graph, plugin, or SDK code | No — `gnuradio4-core` |
| Reusable DSP algorithms and support libraries | No — `gnuradio4-blocks`, under `library/` |
| Standard blocks | No — `gnuradio4-blocks` |
| Incubator, Control Plane, or Studio code | No — its owning repository |
| Module manifest, CMake orchestration, presets, or activation | Yes |
| Cross-component integration tests and installed-SDK smoke tests | Yes |
| Container definitions, top-level CI, or shared workspace documentation | Yes |

Follow the owning component's contribution guidance and issue tracker for a
component change. When a change spans repositories, open the component pull
requests first and link them from the workspace pull request. Test the
workspace against the intended component branches or commits, and describe the
required combination in the pull request.

## Develop and test workspace changes

Start with the development profile for changes to orchestration, documentation,
or the core component stack:

```sh
cmake --preset dev
cmake --build --preset dev --target check
```

Use the full profile when changing the component manifest, Control Plane,
Studio, or integration shared by the complete stack:

```sh
cmake --preset full
cmake --build --preset full --target check
```

The `check` target runs the selected component tests and an out-of-tree
installed-SDK consumer smoke test. The [build guide](docs/building.md),
[testing guide](docs/testing.md), and [CI/container guide](docs/ci.md) describe
the available profiles and the checks expected for a particular type of change.

## DCO sign-off

All commits submitted to this repository must carry a Developer Certificate of
Origin sign-off. The pull-request CI checks every commit for it.

Use `git commit -s` to add the required trailer automatically:

```text
Signed-off-by: Your Name <you@example.com>
```

The complete [Developer Certificate of Origin](DCO.txt) explains what the
sign-off certifies. A DCO sign-off is not a cryptographic signature.

## Pull requests

Describe the problem and the change, identify affected components, and include
the commands and results used to verify it. Keep each commit focused and ensure
you understand and have tested the change you submit.

For changes that affect users, contributors, or CI operators, update the
relevant documentation in the same pull request. For container changes, build
the affected image locally using the documented Docker command; Podman is an
equivalent supported host.

## Licensing

The superbuild's orchestration, CI configuration, helper scripts, and
documentation are licensed under the [MIT License](LICENSE). Each component
repository retains its own license and contribution terms. Contributions must
be made under the license that applies to the files being changed.
