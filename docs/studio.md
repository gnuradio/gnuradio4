# GNU Radio 4 Studio

The `full` profile enables the ZeroMQ, SDR, and Audio block families and adds
control-plane, Studio C++ blocks, and the Node/Vite Studio application. It
builds a desktop Electron bundle and installs it with a pinned Electron
runtime.

## Desktop Studio

```sh
cmake --preset full
cmake --build --preset full
source build/full/activate.sh
gr4-studio-sandbox-setup # needed once on Ubuntu/AppArmor hosts
gr4-studio
```

The Node application target is `gnuradio4-studio`. It shares its source checkout
with the `gnuradio4-studio-blocks` module but does not link against Blocks or
OOT libraries. When control-plane is selected, the superbuild orders it before
Studio. The full desktop build installs `gr4cp_server`, `gr4cp-cli`,
`gr4-studio`, libraries/plugins, and `share/gr4-studio/` into `install/full/`.

Studio waits for a valid `GET /blocks` catalog before opening the canvas. In
local mode, it reserves an available loopback port rather than requiring port
8080. If `gr4cp_server` exits during startup, the launcher reports the backend
log instead of opening an unusable canvas.

On Ubuntu hosts that restrict unprivileged user namespaces through AppArmor,
`gr4-studio-sandbox-setup` installs a path-specific Electron profile and asks
for `sudo`. It remains valid across rebuilds at that prefix. Use
`gr4-studio-sandbox-setup --remove` before permanently moving or deleting the
prefix. Other hosts do not need this step.

Studio can be selected independently with:

```sh
cmake --preset dev -DGR4_MODULES=gnuradio4-studio
```

Studio without control-plane supports a remote backend; select both for the
normal local desktop experience. To connect an installed Studio to a remote
control plane explicitly:

```sh
gr4-studio --remote=http://host.example:8080
```

## Browser Studio image

Pushes to `main` publish:

```text
ghcr.io/gnuradio/gnuradio4-studio:latest
ghcr.io/gnuradio/gnuradio4-studio:sha-<commit>
```

Anonymous pulls require the `gnuradio4-studio` package to be public in GitHub
Container Registry. A repository maintainer may need to set that visibility
after the package is published for the first time.

Run it with Docker:

```sh
docker pull ghcr.io/gnuradio/gnuradio4-studio:latest
docker run --rm -p 8080:8080 ghcr.io/gnuradio/gnuradio4-studio:latest
```

Open <http://localhost:8080>. nginx serves the Studio web bundle and proxies
same-origin `/api/*` requests, including WebSockets, to an internal
control-plane process. The image is intended for trusted development use: it
does not add authentication or TLS termination.

Build the image locally from the workspace root:

```sh
docker build -t gr4-studio-web:local -f containers/studio-web/Dockerfile .
docker run --rm -p 8080:8080 gr4-studio-web:local
```

The build context deliberately excludes local `src/`, `build/`, and `install/`
directories, so a container build uses the component revisions declared in the
superbuild manifest.

Podman supports the same images and commands: replace `docker` with `podman`.
See [Docker and Podman](ci.md#docker-and-podman) for rootless and SELinux mount
differences.
