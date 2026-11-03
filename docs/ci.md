# CI and container policy

## Integration CI

The top-level integration workflow verifies the compatibility contract between
repositories. Pushes to `main`, pull requests, the daily schedule, and manual
runs execute the complete stack on Ubuntu 24.04 with GCC 14:

```sh
cmake --preset full-ci
cmake --build --preset full-ci --target check
```

The run starts by building the third-party dependencies with gr-deps, whose
revision is overridden like any other repository, with `GR4_DEPS_REPOSITORY`
and `GR4_DEPS_REF`.

It exports `CMAKE_BUILD_PARALLEL_LEVEL=4`, limiting nested CMake builds. The
component repositories retain ownership of unit and QA tests as well as broader
compiler, platform, sanitizer, coverage, and WebAssembly matrices. The
superbuild gate covers the installed SDK, plugin discovery, and control-plane
integration; component QA—including incubator QA—runs in its owning repository.

Manual runs accept a space-separated `cmake_options` value for coordinated
branch or fork testing:

```sh
gh workflow run ci.yml \
  -f cmake_options="-DGR4_INCUBATOR_REPOSITORY=https://github.com/me/gr4-incubator.git -DGR4_INCUBATOR_REF=my-branch"
```

Each override must be one shell word. Without overrides, CI follows the module
manifest revisions, normally `main`, to detect compatibility drift.

## Platform containers

The distribution Dockerfiles in `containers/` build the complete workspace
from a clean checkout. They exclude local `src/`, `build/`, and `install/`
trees so the superbuild follows the same first-clone path as a new user.

Each distribution verification image runs:

```sh
cmake --preset full
cmake --build --preset full --target check
```

Use Docker for local builds, matching the GitHub Actions build engine:

```sh
docker build -f containers/debian-sid/Dockerfile -t gr4-test:debian-sid .
docker build -f containers/debian-trixie/Dockerfile -t gr4-test:debian-trixie .
docker build -f containers/ubuntu-24.04/Dockerfile -t gr4-test:ubuntu-24.04 .
docker build -f containers/ubuntu-26.04/Dockerfile -t gr4-test:ubuntu-26.04 .
docker build -f containers/fedora-44/Dockerfile -t gr4-test:fedora-44 .
```

The default `verify` stage performs the check while building the image. For an
interactive toolchain, select its `toolchain` stage and mount a working tree:

```sh
docker build --target toolchain -f containers/debian-sid/Dockerfile \
  -t gr4-toolchain:debian-sid .
docker run --rm -it \
  -v "$PWD:/workspace" -w /workspace \
  gr4-toolchain:debian-sid full
```

The `Platform containers` workflow discovers every Dockerfile in `containers/`
and rebuilds it each Sunday without publishing. This includes the distribution
verification images and the Studio web image. The Debian Sid image is the
rolling compatibility canary; retain its build log and base-image digest when
reporting a regression. Debian Trixie is the stable Debian compatibility target.

## Docker and Podman

Docker is the baseline in this documentation and in GitHub Actions. The images
use standard Dockerfiles and are also supported by Podman. For normal builds
and port publishing, replace `docker` with `podman`:

```sh
podman build -t gr4-studio-web:local -f containers/studio-web/Dockerfile .
podman run --rm -p 8080:8080 gr4-studio-web:local
```

For interactive bind mounts, Podman users should normally add
`--userns=keep-id` to preserve host file ownership. On SELinux hosts, add `:Z`
to bind-mount specifications, for example `-v "$PWD:/workspace:Z"`. Docker
does not use `--userns=keep-id`; use its default container user, or add
`--user "$(id -u):$(id -g)"` when host ownership must be preserved.

## Published Studio image

The integration workflow builds and publishes `gnuradio4-studio` on pushes to
`main`. See [Studio](studio.md) for pull, run, and exposure guidance.
