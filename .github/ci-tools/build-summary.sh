#!/usr/bin/env bash
# Copyright 2026 Marcus Müller
source "$(dirname "$(realpath "$0")")/common.bash"
builddir="$2"
if type -p sccache > /dev/null ; then
  printf '::group::sccache stats\n'
  stats="$(sccache -s)"
  errors="$(printf '%s' "${stats}" | sed -n 's/\(.*\) errors[[:space:]]*\([0-9]*\)$/\1:\2/p')"
  misses="$(printf '%s' "${stats}" | sed -n 's/Cache misses[[:space:]]*\([0-9]*\)$/\1/p')"
  hits="$(printf '%s' "${stats}" | sed -n 's/Cache hits[[:space:]]*\([0-9]*\)$/\1/p')"
  requests="$(printf '%s' "${stats}" | sed -n 's/Compile requests[[:space:]]*\([0-9]*\)$/\1/p')"
  gh_message "Caching" "${misses}:${hits}:${requests} cache misses:cache hits:compile requests"
  for err_line in "${errors}"; do
    count="$(echo "${err_line}" | head -n1 | cut -f2 -d:)"
    if [[ count -gt 0 ]] ; then
      gh_message "$(echo "${err_line}" | cut -f1 -d:)" "${count} errors"
    fi
  done
  if [[ "${misses}" -gt 0 && -r "${SCCACHE_ERROR_LOG}" ]]; then
    printf '::group::sccache misses (N=%d)\n' "${misses}"
    sed -n  "s/.*sccache::compiler::compiler.* \[\(.*\)\]: *Cache miss.*$/\1/p" "${SCCACHE_ERROR_LOG}"
    printf '::endgroup::\n'
  fi
  printf '::endgroup::\n'
else
  gh_message 'skipping sccache stats' 'sccache not found'
fi

printf '::group::build directory stats\n'
kiB_to_MiB () {
  echo "scale=2; $1 / 1024" | bc
}
(
  shopt -s dotglob globstar
  kB_build="$( du -sk "${builddir}" | sed 's/\([^[:space:]]*\).*/\1/')"
  kB_o_files="$( (du -ck "${builddir}"/**/*.o 2> /dev/null || echo 0) | tail -1 | sed 's/\([^[:space:]]*\).*/\1/')"
  kB_so_files="$( (du -ck "${builddir}"/**/*.so 2> /dev/null || echo 0) | tail -1 | sed 's/\([^[:space:]]*\).*/\1/')"
  kB_a_files="$( (du -ck "${builddir}"/**/*.a 2> /dev/null || echo 0) | tail -1 | sed 's/\([^[:space:]]*\).*/\1/')"
  gh_message "Build directory size" "$( kiB_to_MiB "${kB_build}" ) MiB"
  gh_message "Build directory shared library (.so) files cumulative size" "$( kiB_to_MiB "${kB_so_files}" ) MiB"
  gh_message "Build directory object (.o) files cumulative size" "$( kiB_to_MiB "${kB_o_files}" ) MiB"
  gh_message "Build directory static library (.a) files cumulative size" "$( kiB_to_MiB "${kB_a_files}" ) MiB"
)
printf '::endgroup::\n'
