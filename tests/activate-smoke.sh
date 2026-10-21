#!/bin/sh
# SPDX-License-Identifier: MIT
#
# Renders cmake/activate.sh.in against throw-away prefixes and sources it in
# every shell found on the host. zsh needs particular care: it aborts a command
# whose glob matches nothing, which must not break activation or leave
# PYTHONPATH unset.

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
template="$script_dir/../cmake/activate.sh.in"

if [ ! -f "$template" ]; then
  echo "activate.sh.in not found: $template" >&2
  exit 2
fi

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT INT TERM

failures=0
shells_run=0

fail() {
  echo "FAIL [$shell_name / $case_name]: $*" >&2
  failures=$((failures + 1))
}

# render <prefix> -> path of a generated activate script
render() {
  out="$1/activate.sh"
  sed \
    -e "s|@CMAKE_BINARY_DIR@|$1/build|g" \
    -e "s|@CMAKE_INSTALL_PREFIX@|$1|g" \
    -e "s|@GR4_INSTALL_LIBDIR_FULL@|$1/lib|g" \
    "$template" >"$out"
  printf '%s\n' "$out"
}

# source_in <shell> <script> -> prints PYTHONPATH, GR4_PREFIX and the leading PATH entry;
# stderr is captured in $errfile.
source_in() {
  env -i PATH=/usr/bin:/bin HOME="$tmpdir" "$1" -c '
    . "$1"
    printf "PYTHONPATH=%s\n" "${PYTHONPATH:-}"
    printf "GR4_PREFIX=%s\n" "${GR4_PREFIX:-}"
    printf "PATH_FIRST=%s\n" "${PATH%%:*}"
    if type _gr4_add_python_paths >/dev/null 2>&1; then echo "HELPER_LEAKED=yes"; fi
    if [ -n "${ZSH_VERSION:-}" ]; then
      if [[ -o nullglob ]]; then echo "NULLGLOB_LEAKED=yes"; fi
    fi
  ' sh "$2" 2>"$errfile"
}

# check_case <description> <prefix directory name> [directory below lib/ ...]
check_case() {
  case_name="$1"
  prefix="$tmpdir/$shell_name-$2"
  shift 2
  mkdir -p "$prefix/lib"
  for dir in "$@"; do
    mkdir -p "$prefix/lib/$dir"
  done
  script=$(render "$prefix")
  errfile="$tmpdir/stderr"
  if ! output=$(source_in "$shell_path" "$script"); then
    fail "sourcing the script failed: $(cat "$errfile")"
    return
  fi
  if [ -s "$errfile" ]; then
    fail "unexpected stderr: $(cat "$errfile")"
  fi
  printf '%s\n' "$output" | grep -Fxq "GR4_PREFIX=$prefix" || fail "GR4_PREFIX not set to $prefix"
  printf '%s\n' "$output" | grep -Fxq "PATH_FIRST=$prefix/bin" || fail "$prefix/bin is not first on PATH"
  printf '%s\n' "$output" | grep -Fq "HELPER_LEAKED" && fail "helper function left defined"
  printf '%s\n' "$output" | grep -Fq "NULLGLOB_LEAKED" && fail "NULL_GLOB leaked into the sourcing shell"
  pythonpath=$(printf '%s\n' "$output" | sed -n 's/^PYTHONPATH=//p')
  for dir in "$@"; do
    case "$dir" in
      python*/site-packages | python*/dist-packages)
        case ":$pythonpath:" in
          *":$prefix/lib/$dir:"*) ;;
          *) fail "PYTHONPATH is missing $prefix/lib/$dir (got '$pythonpath')" ;;
        esac
        ;;
    esac
  done
  if [ "$#" -eq 0 ] && [ -n "$pythonpath" ]; then
    fail "PYTHONPATH should be empty without Python directories (got '$pythonpath')"
  fi
}

for shell_name in sh bash dash zsh; do
  shell_path=$(command -v "$shell_name" 2>/dev/null || true)
  if [ -z "$shell_path" ]; then
    echo "skip: $shell_name not installed"
    continue
  fi
  shells_run=$((shells_run + 1))
  check_case "no python directories" none
  check_case "site-packages only" site python3.12/site-packages
  check_case "dist-packages only" dist python3.12/dist-packages
  check_case "several versions" multi python3.11/site-packages python3.12/dist-packages
  check_case "prefix with spaces" "with space" python3.12/site-packages
done

if [ "$shells_run" -eq 0 ]; then
  echo "no supported shell found" >&2
  exit 2
fi

if [ "$failures" -ne 0 ]; then
  echo "$failures check(s) failed" >&2
  exit 1
fi

echo "activate.sh smoke test passed in $shells_run shell(s)"
