[![Main CI](https://github.com/gnuradio/gnuradio4/actions/workflows/ci.yml/badge.svg?branch=main&event=push)](https://github.com/gnuradio/gnuradio4/actions/workflows/ci.yml)
[![Studio Web image](https://img.shields.io/badge/ghcr.io-gnuradio4--studio-2496ED?logo=github)](https://github.com/gnuradio/gnuradio4/pkgs/container/gnuradio4-studio)

<p align="center">
  <img
    src="https://raw.githubusercontent.com/gnuradio/gnuradio4-core/main/docs/logo.png"
    alt="GNU Radio 4"
    width="30%">
</p>

# GNU Radio 4

This is the official repository and entry point for GNU Radio 4 (GR4).
[GNU Radio](https://www.gnuradio.org) is a free and open-source signal-processing runtime and software
development toolkit. GR4 provides a modern C++23 block API, runtime-loadable
blocks and schedulers, an installed development SDK, and browser and desktop
Studio applications. This repository is the top-level GR4 workspace: it builds
the independently maintained core, library, blocks, incubator, control-plane,
and Studio repositories in dependency order into one development prefix.

## Requirements

The core stack requires CMake 3.27+, Ninja, Git, and a C++23 compiler (GCC 14+
or a current Clang). The complete Studio workspace also requires Node.js 22 and
npm. See [Build guide](docs/building.md) for platform setup, profile choices,
and resource limits.

## Quick start

Clone the workspace and build the core development stack:

```sh
git clone https://github.com/gnuradio/gnuradio4.git
cd gnuradio4
cmake --preset dev
cmake --build --preset dev
source build/dev/activate.sh
```

Build the complete workspace, including incubator, control-plane, and the
desktop Studio application:

```sh
cmake --preset full
cmake --build --preset full
source build/full/activate.sh
gr4-studio
```

On Ubuntu hosts with restricted unprivileged user namespaces, run
`gr4-studio-sandbox-setup` once before launching Studio. See the
[Studio guide](docs/studio.md#desktop-studio) for details.

Run the published browser Studio image instead:

```sh
docker pull ghcr.io/gnuradio/gnuradio4-studio:latest
docker run --rm -p 8080:8080 ghcr.io/gnuradio/gnuradio4-studio:latest
```

Then visit <http://localhost:8080>.

The initial build clones missing component repositories into `src/`. After a
checkout exists, the superbuild leaves its Git state alone; you choose when to
fetch, update, or switch its branch.

Docker is the documented container host. Podman is supported as a rootless
alternative; see [CI and containers](docs/ci.md#docker-and-podman).

## Workspace components

| Component | Purpose |
| --- | --- |
| `gnuradio4-core` | Runtime, scheduler, graph/block model, plugins, and SDK |
| `gnuradio4-library` | Reusable DSP algorithms and support libraries |
| `gnuradio4-blocks` | Standard signal-processing and utility blocks |
| `gr4-incubator` | Experimental blocks, schedulers, and utilities |
| `gnuradio4-control-plane` | Runtime and HTTP control service |
| `gnuradio4-studio` | Studio blocks plus browser and desktop application |

## Repository dependencies

```text
Foundation
  [gnuradio4-core]     [gnuradio4-library]
            \             /
             \           /
Modules       [blocks] [incubator] [OOTs]
                  .        .         .
                  `------ runtime plugins ------.
                                                 v
Applications                          [control-plane] --> [studio]
```

Core and library are build dependencies of blocks, incubator, and OOTs; the
control plane depends on core at build time. Dotted paths are runtime plugin
discovery.

The superbuild coordinates the repositories it includes; OOTs remain
independently owned and can be added to the workspace as described in the
[extending guide](docs/extending.md).

## Documentation

| Task | Documentation |
| --- | --- |
| Configure, build, select a profile, or use macOS | [Build guide](docs/building.md) |
| Select modules or register a workspace module | [Module configuration](docs/modules.md) |
| Launch desktop or browser Studio | [Studio guide](docs/studio.md) |
| Run component and integration tests | [Testing guide](docs/testing.md) |
| Develop a child repository or add an out-of-tree project | [Extending the workspace](docs/extending.md) |
| Understand CI and test/container image policy | [CI and containers](docs/ci.md) |
| Understand the superbuild's architectural choices | [Design](DESIGN.md) |

## Common commands

```sh
# Run the selected stack's tests and installed-SDK smoke test.
cmake --build --preset dev --target check

# List the supplied configure and build profiles.
cmake --list-presets
cmake --build --list-presets

# Build only standard blocks (required dependencies are selected automatically).
cmake --build --preset dev --target gnuradio4-blocks
```

## Contributing

The repositories under `src/` are ordinary Git working copies. Make component
changes in their owning repository and follow that repository's contribution
guidance. This workspace owns cross-component build orchestration, integration
CI, and shared development workflows.

## Helpful links

- [GNU Radio website](https://gnuradio.org/)
- [GNU Radio wiki](https://wiki.gnuradio.org/)
- [GNU Radio Matrix chat](https://chat.gnuradio.org/)
- [GNU Radio 4 technical discussion](https://matrix.to/#/#gr4-technical-users:gnuradio.org)

## Acknowledgements

GNU Radio acknowledges GSI/FAIR (Facility for Antiproton and Ion Research,
Darmstadt, Germany) and the wider GNU Radio community for their contributions
to the design and development of GNU Radio 4.0.

## License

The superrepo's CMake orchestration, CI configuration, helper scripts, and
documentation are distributed under the [MIT License](LICENSE). Individual
component repositories retain their own authoritative licensing and
contribution terms.
