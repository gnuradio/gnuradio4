<p align="center">
<img src="docs/logo.png" width="30%" />
</p>

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![CI](https://github.com/gnuradio/gnuradio4/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/gnuradio/gnuradio4/actions/workflows/ci.yml)

<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->

[![All Contributors](https://img.shields.io/badge/all_contributors-18-orange.svg?style=flat-square)](#contributors-)

<!-- ALL-CONTRIBUTORS-BADGE:END -->

# GNU Radio 4.0

> [!IMPORTANT]
> GNU Radio 4.0 (GR4) is currently in a maturing beta state as it approaches its
> first stable release. It is suitable for evaluation, experimentation, and early
> development. GNU Radio 3.x remains the current stable release series for users
> who require the existing production-supported GNU Radio platform.
>
> - GNU Radio 3.x stable release series: https://github.com/gnuradio/gnuradio
> - Report GR4 issues here: https://github.com/gnuradio/gnuradio4/issues
> - Report GNU Radio 3.x issues here: https://github.com/gnuradio/gnuradio/issues

GNU Radio is a free & open-source signal processing runtime and signal processing
software development toolkit. Originally developed for use with software-defined
radios and for simulating wireless communications, it's robust capabilities have
led to adoption in hobbyist, academic, and commercial environments. GNU Radio has
found use in software-defined radio, digital communications, nuclear physics, high-
energy particle physics, astrophysics, radio astronomy and more!

## Building

GNU Radio 4.0 uses modern C++ (C++23), and is tested for

- CMake (>= 3.25),
- GCC (>=14, recommended: >=15)
- Clang (>=20, recommended), and
- Emscripten (5.0.2).

**To build**:

```bash
git clone https://github.com/gnuradio/gnuradio4.git
cd gnuradio4

# (Optional) If you experience excessive gcc memory usage during builds (needs sudo):
sudo ./enableZRAM.sh

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithAssert -DGR_ENABLE_BLOCK_REGISTRY=ON ..
cmake --build . -- -j$(nproc)
```

**Cleaning up zram** if used:

```bash
sudo swapoff /dev/zram0
echo 1 | sudo tee /sys/block/zram0/reset
```

### Key CMake Flags `-D...=<ON|OFF>`

- **`GR_ENABLE_BLOCK_REGISTRY`** (default: ON): enables a runtime registry of blocks.
  Turning this off gives fully static builds.
- **`EMBEDDED`** (default: OFF): reduces code size and runtime features for constrained systems.
  Also implicitly enabled by `-DCMAKE_BUILD_TYPE=MinSizeRel`.
- **`WARNINGS_AS_ERRORS`** (default: ON): treats all compiler warnings as errors (`-Werror`).
- **`TIMETRACE`** (default: OFF): activates Clang’s `-ftime-trace` for per-file compilation timing.
- **`ADDRESS_SANITIZER`** (default: OFF): enables AddressSanitizer (can’t be combined with the other sanitiser options).
- **`UB_SANITIZER`** (default: OFF): enables 'Undefined Behavior' checks.
- **`THREAD_SANITIZER`** (default: OFF): enables threading checks (N.B. strong impact on performance).

### Example Combined Command

```bash
cmake -B build -S . \
  -DCMAKE_BUILD_TYPE=RelWithAssert \
  -DGR_ENABLE_BLOCK_REGISTRY=ON \
  -DWARNINGS_AS_ERRORS=ON \
  -DTIMETRACE=OFF \
  -DADDRESS_SANITIZER=OFF \
  -DUB_SANITIZER=OFF \
  -DTHREAD_SANITIZER=OFF
cmake --build build -- -j$(nproc)
```

Feel free to tweak these flags based on your needs (embedded targets, debugging, sanitising, etc.).
For more details, see [DEVELOPMENT.md](DEVELOPMENT.md) or comments in the `CMakeLists.txt` file that
describe how to set up a local development environment.

## Helpful Links

- [GNU Radio Website](https://gnuradio.org)
- [GNU Radio Wiki](https://wiki.gnuradio.org/)
- [Github issue tracker for bug reports and feature requests](https://github.com/gnuradio/gnuradio4/issues)
- [View the GNU Radio Mailing List Archive](https://lists.gnu.org/archive/html/discuss-gnuradio/)
- [Subscribe to the GNU Radio Mailing List](https://lists.gnu.org/mailman/listinfo/discuss-gnuradio)
- [GNU Radio Chatroom on Matrix](https://chat.gnuradio.org/)
  - Specifically for discussions related to GNU Radio 4.0 join the [#architecture channel](https://matrix.to/#/#gr4-technical-users:gnuradio.org)

## What's New in GNU Radio 4.0?

GNU Radio 4.0 is a major modernization of the GNU Radio runtime, block model, and application architecture. It preserves the core GNU Radio workflow - building signal-processing systems from reusable blocks and flowgraphs - while introducing a cleaner C++ foundation, stronger typing, improved performance, and more flexible runtime behavior.

- **Familiar GNU Radio Workflow**: Blocks and flowgraphs remain central to GNU Radio. Applications can still be built graphically, from Python, or directly in C++, while the underlying architecture has been simplified and modernized.

- **Modern C++ Block Development**: GNU Radio 4 uses modern C++ language features and design patterns to make block development more direct, type-safe, and maintainable.

- **Stronger Data Type Support**: GR4 supports fundamental numeric types such as integers, floats, and complex values, while also enabling structured, user-defined, and application-specific data types.

- **High-Performance Runtime**: The runtime is designed for efficient signal processing using lock-free buffers, compile-time optimization, and SIMD support

- **Flexible Scheduling Model**: GR4 introduces a more flexible scheduling architecture, allowing different schedulers to optimize for throughput, latency, parallelism, or application-specific execution requirements.

- **Recursive Flowgraphs and Feedback**: Flowgraphs can represent recursive directed graphs, enabling feedback loops and more expressive system architectures.

- **Broader Execution Targets**: GR4 is designed with portability in mind, targeting CPUs today while leaving room for hardware accelerators and heterogeneous architectures

- **From Research to Deployment**: GNU Radio 4 aims to serve the full SDR lifecycle: experimentation, education, prototyping, test systems, and operational research or industrial deployments.

## License and Copyright

Unless otherwise noted: SPDX-License-Identifier: MIT<br>
All code contributions to GNU Radio core and runtime will be integrated into a library under the MIT, ensuring it remains free/libre (FLOSS) for both personal and commercial use, without further constraints on either. GNU Radio also allows for individual block libraries to be licensed as GPLv3.
For details on these distinctions and how to contribute, please consult: [CONTRIBUTING.md](CONTRIBUTING.md)

Copyright (C) The GNU Radio Authors<br>
Copyright (C) Contributors to the GNU Radio Project<br>
Copyright (C) FAIR - Facility for Antiproton & Ion Research, Darmstadt, Germany<br>

## Acknowledgements

The GNU Radio project appreciates the contributions from GSI/FAIR in the co-development of GNU Radio 4.0. Their dedicated efforts have played a key role in enhancing the capabilities of our open-source SDR technology.
We would like to recognize the following contributors for their roles in redesigning the core that has evolved into GR 4.0:

## Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/wirew0rm"><img src="https://avatars.githubusercontent.com/u/1202371?v=4" width="100px;" alt=""/><br /><sub><b>Alexander Krimm</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/noc0lour"><img src="https://avatars.githubusercontent.com/u/4438327?v=4" width="100px;" alt=""/><br /><sub><b>Andrej Rode</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/gitXsingh"><img src="https://avatars.githubusercontent.com/u/149612072?v=4" width="100px;" alt=""/><br /><sub><b>Anmolmeet Singh</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/cafeclimber"><img src="https://avatars.githubusercontent.com/u/10188900?v=4" width="100px;" alt=""/><br /><sub><b>Bailey Campbell</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/chrisjohgorman"><img src="https://avatars.githubusercontent.com/u/29354995?v=4" width="100px;" alt=""/><br /><sub><b>Chris Gorman</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="http://destevez.net"><img src="https://avatars.githubusercontent.com/u/15093841?v=4" width="100px;" alt=""/><br /><sub><b>Daniel Estévez</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/dennisklein"><img src="https://avatars.githubusercontent.com/u/297548?v=4" width="100px;" alt=""/><br /><sub><b>Dennis Klein</b></sub></a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/frankosterfeld"><img src="https://avatars.githubusercontent.com/u/483854?v=4" width="100px;" alt=""/><br /><sub><b>Frank Osterfeld</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="http://cukic.co"><img src="https://avatars.githubusercontent.com/u/90119?v=4" width="100px;" alt=""/><br /><sub><b>Ivan Čukić</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/marcusmueller"><img src="https://avatars.githubusercontent.com/u/958972?v=4" width="100px;" alt=""/><br /><sub><b>Marcus Müller</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://mattkretz.github.io/"><img src="https://avatars.githubusercontent.com/u/3306474?v=4" width="100px;" alt=""/><br /><sub><b>Matthias Kretz</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/eltos"><img src="https://avatars.githubusercontent.com/u/19860638?v=4" width="100px;" alt=""/><br /><sub><b>Philipp Niedermayer</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/RalphSteinhagen"><img src="https://avatars.githubusercontent.com/u/46007894?v=4" width="100px;" alt=""/><br /><sub><b>Ralph J. Steinhagen</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/iamsergio"><img src="https://avatars.githubusercontent.com/u/20387?v=4" width="100px;" alt=""/><br /><sub><b>Sergio Martins</b></sub></a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/flynn378"><img src="https://avatars.githubusercontent.com/u/6114517?v=4" width="100px;" alt=""/><br /><sub><b>Toby Flynn</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/gretel"><img src="https://avatars.githubusercontent.com/u/80815?v=4" width="100px;" alt=""/><br /><sub><b>Tom Hensel</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/drslebedev"><img src="https://avatars.githubusercontent.com/u/25366186?v=4" width="100px;" alt=""/><br /><sub><b>drslebedev</b></sub></a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/mormj"><img src="https://avatars.githubusercontent.com/u/34754695?v=4" width="100px;" alt=""/><br /><sub><b>mormj</b></sub></a></td>
    </tr>
  </tbody>
  <tfoot>
    <tr>
      <td align="center" size="13px" colspan="7">
        <img src="https://raw.githubusercontent.com/all-contributors/all-contributors-cli/1b8533af435da9854653492b1327a23a4dbd0a10/assets/logo-small.svg">
          <a href="https://all-contributors.js.org/docs/en/bot/usage">Add your contributions</a>
        </img>
      </td>
    </tr>
  </tfoot>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->
