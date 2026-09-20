# Packaging

The superbuild can build one installer covering every selected module. It is off
by default:

```bash
cmake --preset <preset> -DGR_ENABLE_PACKAGING=ON
cmake --build build/<preset> --target gr4-installer
```

Supported on Windows and macOS. On other platforms `GR_ENABLE_PACKAGING` prints
a note and adds no targets; each module still builds its own `.deb` through its
own `package` target.

The superbuild never repackages a module's files. Each module knows how to
package itself, and the bundle composes what they produce.

| | Windows | macOS |
|---|---|---|
| Module artefact | one Inno Setup `.exe` per module | one flat `.pkg` per CPack component |
| Bundle | embeds the installers, runs the selected ones silently | a `productbuild` distribution package referencing the component packages |
| Output | `gnuradio4-<version>-win64.exe` | `gnuradio4-<version>-macos.pkg` |
| Tool | `ISCC.exe` (Inno Setup 6) | `productbuild`, `pkgbuild` (Xcode command line tools) |

The per-module machinery lives in gr-core, not here: `cmake/GrInnoSetup.cmake`
and `cmake/GrProductBuild.cmake`, with their templates under
`packaging/innosetup/` and `packaging/productbuild/`. Both are installed into
`<prefix>/lib/cmake/gnuradio4/` so every downstream module and the superbuild
read the same copy.

## Targets

| Target | What it does |
|--------|--------------|
| `<module>-package` | builds one module's artefacts into `<build>/packaging/components` |
| `gr4-installer` | builds every `<module>-package`, then composes the bundle |
| `package` | alias for `gr4-installer` when nothing else defines it |

A module that has no installer configuration is skipped with a status message
rather than failing the build, which is how `gr4-incubator`,
`gnuradio4-control-plane` and the studio modules stay out of the bundle.

## Options

| Option | Default | Meaning |
|--------|---------|---------|
| `GR_ENABLE_PACKAGING` | `OFF` | master switch; also passed to every module as `GR_PACKAGING` |
| `GR4_BUNDLE_VERSION` | `4.0.0` | version of the unified installer |
| `GR4_BUNDLE_APP_ID` | a GUID | Windows only; Inno Setup `AppId`. Never change it between releases |
| `GR4_BUNDLE_IDENTIFIER` | `org.gnuradio.gr4.bundle` | macOS only; `pkgutil` identifier. Never change it between releases |
| `GR4_MACOS_MIN_VERSION` | `13.0` | minimum macOS the installer allows |
| `GR4_MACOS_HOST_ARCHITECTURES` | `arm64,x86_64` | architectures the installer may run on |
| `GR4_MACOS_SIGN_IDENTITY` | empty | Developer ID Installer identity; empty leaves the package unsigned |
| `GR4_MACOS_NOTARIZE_PROFILE` | empty | `notarytool` keychain profile; empty skips notarisation |
| `GR4_ISCC_EXECUTABLE` | found | path to the Inno Setup compiler |

`GR4_MACOS_SIGN_IDENTITY` is forwarded to the module builds, so the component
packages and the bundle composed around them carry the same signature. The
signature Gatekeeper actually checks is the one on the bundle; an unsigned
package still installs after a right-click → Open.

## The shared install manifest

Both platforms keep one JSON manifest per plugin API version recording where GNU
Radio 4 lives and which packages and plugins are installed. It is the same file
format on both, so one description covers them.

```json
{
  "schema": 1,
  "api-version": "1",
  "install-root": "/usr/local/gnuradio4",
  "plugin-root": "/Library/Application Support/.gnuradio-4/plugins/v1",
  "components": { "gr-core": { "version": "4.0.0" } },
  "plugins": {}
}
```

Two words collide here, and the distinction matters:

- **package** — a versioned GR product recorded in the manifest, e.g.
  `gr-core=4.0.0`. Kept in the `components` and `plugins` sections.
- **component** — a CPack install component, e.g. `CORE` or `DEVELOP`: a
  separately selectable part of one installer. One package normally spans
  several components.

`<api>` is `GR_PLUGIN_API_VERSION`, read from `Plugin.hpp` by gr-core. One
manifest per API version, so two ABIs can be installed side by side without
seeing each other's entries.

### Layout

Windows, per-user, no elevation:

```
%LOCALAPPDATA%\Programs\GnuRadio4            install root (user-selectable)
%LOCALAPPDATA%\.gnuradio-4\manifest\v<api>   manifest.json
%LOCALAPPDATA%\.gnuradio-4\plugins\v<api>    plugin root
%LOCALAPPDATA%\.gnuradio-4\tools             gr_manifest.ps1
```

macOS, system domain, admin-authenticated:

```
/usr/local/gnuradio4                                       install root
/usr/local/gnuradio4/bin/gr4-uninstall                     the uninstaller
/Library/Application Support/.gnuradio-4/manifest/v<api>   manifest.json
/Library/Application Support/.gnuradio-4/plugins/v<api>    plugin root
/Library/Application Support/.gnuradio-4/tools             gr_manifest.sh
```

The manifest tool is `gr_manifest.ps1` (PowerShell) on Windows and
`gr_manifest.sh` (`/bin/sh` plus `plutil`, both present on every macOS) on
macOS. They take the same `|`-separated `name=version` inputs and emit the same
`key=value` result lines; keep them in step.

The macOS tool writes one extra per-entry field, `identifier-prefix`, recording
the `pkgutil` identifier so `gr4-uninstall` can find the receipts to forget.
Readers must treat it as optional — the PowerShell tool never writes it.

## Uninstalling

Windows gives every module an entry in Apps & Features. macOS has no uninstaller
concept, so gr-core ships one:

```bash
gr4-uninstall --list                  # what is installed, and where
sudo gr4-uninstall --package gr-blocks
sudo gr4-uninstall --all
sudo gr4-uninstall --all --dry-run    # print what --all would remove
```

It removes each receipt's payload, runs `pkgutil --forget`, and unregisters the
package from the manifest. The shared tree under
`/Library/Application Support/.gnuradio-4` is pruned only when the last manifest
entry goes — plugins may deliberately outlive the runtime — and only with
`rmdir`, which refuses non-empty directories, so nothing a user put there is
removed.

## Where macOS deliberately differs from Windows

Four things do not carry over. They are design decisions, not gaps to be closed
later by making macOS imitate Windows.

1. **No install types.** The Windows installer's `[Types]` dropdown (`Base`,
   `Custom`) comes from the module `GROUPS`. macOS Installer has no equivalent,
   so the groups decide which choices start ticked instead: a module in the
   `base` group starts selected, everything else starts unselected, and the
   Customize pane is always available.
2. **No install-root negotiation.** A `.pkg`'s install location is fixed when it
   is built. The Windows flow — the first installer picks a root, later ones
   inherit it, `/DIR` is refused once a manifest exists — has nothing to
   negotiate on macOS. The manifest still *records* `install-root`, which is what
   third-party plugin installers read.
3. **Downgrades are refused, not confirmed.** Inno Setup asks the user and
   defaults to No. A `.pkg` preinstall script has no UI: it can only proceed or
   fail. It fails, preserving the guarantee the Windows prompt protects — never
   silently replace newer files with older ones and break plugins built against
   the newer runtime. The reason is written to `/var/log/install.log`. Set
   `GR4_ALLOW_DOWNGRADE=1` in the environment to override.
4. **Dependencies are not enforced** on either platform: installing blocks
   without core is allowed. macOS `<choice>` could express it and deliberately
   does not, so the two behave the same.

## Known limitations

- **A macOS package expects Homebrew LLVM.** `RUNTIME_DEPENDENCIES` excludes
  `/opt/homebrew` and `/usr/local/opt`, because bundling those dylibs would copy
  them in without rewriting any install name — the binaries would still load the
  originals and the copies would be dead weight. Making the package
  self-contained needs an `install_name_tool` fixup step that does not exist yet.
- **Nothing reads the manifest at runtime.** `PluginLoader` only consults
  `GNURADIO4_PLUGIN_DIRECTORIES`. The installers write `plugin-root` and no
  shipped code reads it back, on either platform, so an installed GR 4 finds no
  plugins unless that variable is set by hand.
- **Plugins installed into the shared plugin tree have no usable RPATH.**
  `@loader_path/../lib` is correct for anything under the install prefix, but a
  plugin in `/Library/Application Support/.gnuradio-4/plugins/v<api>` needs an
  absolute RPATH into `<install-root>/lib`, which nothing sets.
- **Universal binaries are not built.** The installer declares
  `arm64,x86_64` as host architectures, but what lands in it is whatever
  `CMAKE_OSX_ARCHITECTURES` produced — a single-architecture build by default.
- **No CI coverage.** Neither the Windows bundle nor the macOS one is built in
  CI; superbuild CI is Linux-only.
