# Script mode: stages the dependency headers the installed GR 4 needs into a
# tree the bundle ships.
#
#   cmake -DPREFIXES=a|b|c -DLIBDIR=lib -DOUT_DIR=... -P StageDependencyPayload.cmake
#
# GR 4's installed CMake package config find_dependency()s these packages, so a
# consumer holding nothing but an installed GR 4 cannot get through
# find_package(gnuradio4) without them. They are not extras and not a choice:
# they travel with GR's development files or those files do not work.
#
# They are also not GR's own code. They come out of the dependency build, and
# this is the superbuild, which is the one place that knows which prefix each
# one was built into. That is why the staging lives here rather than in a
# module: no module owns these files, and none should install them.
cmake_minimum_required(VERSION 3.25)

# Extra places to look for a package's licence text. Needed because the
# dependency build installs headers and package configs but not licences, so
# the prefix alone cannot answer "under what terms". Each entry is searched for
# <dir>/<package>/LICENSE and friends, and for a source checkout's own root.
set(LICENSE_DIRS "" CACHE STRING "Extra directories to search for licence texts")

foreach(variable PREFIXES LIBDIR OUT_DIR)
  if(NOT DEFINED ${variable} OR "${${variable}}" STREQUAL "")
    message(FATAL_ERROR "StageDependencyPayload.cmake requires -D${variable}=...")
  endif()
endforeach()
string(REPLACE "|" ";" prefixes "${PREFIXES}")
string(REPLACE "|" ";" licence_dirs "${LICENSE_DIRS}")

# DEPS_PREFIX is the prefix the superbuild built the dependencies into. When it
# is set it is the answer rather than a candidate: it is searched alone, and a
# package missing from it is an error that names it.
#
# Searching a list that begins with GR's own install prefix is wrong in two
# ways. That prefix is GR's output, and holds dependencies only when the
# dependency build happened to share it, which is a default rather than a
# fact. And a search cannot tell a package this build produced from one an
# earlier run left behind, so a stale dependency is found and shipped with
# nothing to say it is stale.
#
# The list stays for the build whose dependencies were not built here at all
# and come from the system, Spack or vcpkg. Searching is the only option there.
if(DEFINED DEPS_PREFIX AND NOT "${DEPS_PREFIX}" STREQUAL "")
  set(prefixes "${DEPS_PREFIX}")
  set(prefix_source "the dependency build's prefix")
else()
  set(prefix_source "the prefix search list")
endif()

# Enumerated, never globbed. Each entry is a package gnuradio4Config.cmake
# names, so this list is checkable against that file rather than against
# whatever happens to be sitting in a prefix.
#
#   package | headers it installs | licence
set(payload_packages magic_enum vir-simd ut)
set(payload_headers_magic_enum "magic_enum*.hpp")
set(payload_headers_vir-simd "vir")
set(payload_headers_ut "boost")
set(payload_licence_magic_enum "MIT")
set(payload_licence_vir-simd "LGPL-3.0-or-later")
set(payload_licence_ut "BSL-1.0")

# cpp-httplib and OpenSSL are deliberately absent: core find_dependency()s
# httplib only for its own tests, and shipping OpenSSL is a runtime and
# security decision rather than a development one.

# Finds one package along the prefix list, in order, the way find_package
# searches. With DEPS_PREFIX set that list holds exactly one entry, so this
# resolves to the dependency build's prefix or to nothing.
function(stage_find_package package cmake_output prefix_output)
  foreach(prefix IN LISTS prefixes)
    if(IS_DIRECTORY "${prefix}/${LIBDIR}/cmake/${package}")
      set(${cmake_output} "${prefix}/${LIBDIR}/cmake/${package}" PARENT_SCOPE)
      set(${prefix_output} "${prefix}" PARENT_SCOPE)
      return()
    endif()
    # a header-only package may install its config under share/ instead
    if(IS_DIRECTORY "${prefix}/share/cmake/${package}")
      set(${cmake_output} "${prefix}/share/cmake/${package}" PARENT_SCOPE)
      set(${prefix_output} "${prefix}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  set(${cmake_output} "" PARENT_SCOPE)
  set(${prefix_output} "" PARENT_SCOPE)
endfunction()

file(REMOVE_RECURSE "${OUT_DIR}")
file(MAKE_DIRECTORY "${OUT_DIR}")

set(staged "")
set(missing "")
foreach(package IN LISTS payload_packages)
  stage_find_package("${package}" cmake_dir package_prefix)
  if(NOT cmake_dir)
    list(APPEND missing "${package}")
    continue()
  endif()

  # the package config keeps the directory it was found in, so a consumer's
  # find_package() resolves it the same way it would in the source prefix
  cmake_path(GET cmake_dir PARENT_PATH cmake_parent)
  cmake_path(RELATIVE_PATH cmake_parent BASE_DIRECTORY "${package_prefix}" OUTPUT_VARIABLE cmake_relative)
  file(COPY "${cmake_dir}" DESTINATION "${OUT_DIR}/${cmake_relative}")

  # headers: a directory for the packages that namespace themselves, a glob
  # for magic_enum, which puts its headers at the top of include/
  set(pattern "${payload_headers_${package}}")
  if(IS_DIRECTORY "${package_prefix}/include/${pattern}")
    file(COPY "${package_prefix}/include/${pattern}" DESTINATION "${OUT_DIR}/include")
  else()
    file(GLOB header_files "${package_prefix}/include/${pattern}")
    if(NOT header_files)
      message(FATAL_ERROR "dependency payload: ${package} has a package config but no headers matching ${pattern}")
    endif()
    file(COPY ${header_files} DESTINATION "${OUT_DIR}/include")
  endif()

  # The licence travels with the headers. vir-simd is LGPL-3.0-or-later, so for
  # that one this is an obligation rather than a courtesy, and the headers GR
  # carries are a modified version of it.
  # Prefer the package's whole licence directory when the dependency build
  # provides one. For vir-simd that directory is not one file: LGPL-3.0 is
  # GPL-3.0 plus added permissions, so both texts have to travel, and the
  # record of GR's modifications travels with them. Copying a single LICENSE
  # would silently drop all of that.
  set(licence_dir "${package_prefix}/share/licenses/${package}")
  if(IS_DIRECTORY "${licence_dir}")
    file(COPY "${licence_dir}" DESTINATION "${OUT_DIR}/share/licenses")
    list(APPEND staged "${package}")
    continue()
  endif()

  set(licence_file "")
  set(licence_search "${package_prefix}/share/licenses/${package}" "${package_prefix}/share/${package}")
  # LICENSE_DIRS entries are package-qualified, "<package>=<directory>". They
  # were once bare directories searched for every package, which meant the
  # first one answered for all of them and each package was shipped under
  # someone else's licence. A wrong licence is worse than a missing one: the
  # missing one fails the build, the wrong one ships.
  foreach(entry IN LISTS licence_dirs)
    if(NOT entry MATCHES "^([^=]+)=(.+)$")
      message(FATAL_ERROR "LICENSE_DIRS entry is not <package>=<directory>: ${entry}")
    endif()
    if(CMAKE_MATCH_1 STREQUAL package)
      list(APPEND licence_search "${CMAKE_MATCH_2}")
    endif()
  endforeach()
  foreach(directory IN LISTS licence_search)
    foreach(candidate LICENSE LICENSE.txt LICENSE.md COPYING COPYING.LESSER)
      if(EXISTS "${directory}/${candidate}" AND NOT licence_file)
        set(licence_file "${directory}/${candidate}")
      endif()
    endforeach()
  endforeach()
  if(NOT licence_file)
    # Not a warning. Shipping someone's code without their licence is not a
    # tidiness problem, and for vir-simd, which is LGPL-3.0-or-later and is
    # carried with modifications, it is a licence violation. Fail here rather
    # than produce an installer that cannot lawfully be handed to anyone.
    message(FATAL_ERROR
      "dependency payload: no licence text for ${package} (${payload_licence_${package}}).
"
      "Searched: ${licence_search}
"
      "Pass -DLICENSE_DIRS=... with a directory holding it, or install it alongside "
      "the package.")
  endif()
  file(COPY "${licence_file}" DESTINATION "${OUT_DIR}/share/licenses/${package}")

  # One licence file is not always the whole obligation. A source tree may
  # carry a LICENSES/ directory holding every text its headers refer to, and a
  # record of local modifications; for vir-simd both matter, because LGPL-3.0
  # incorporates GPL-3.0 by reference and GR patches the headers. Take them
  # when they are there, so DEPENDENCIES.txt does not promise a text that was
  # left behind.
  cmake_path(GET licence_file PARENT_PATH licence_origin)
  if(IS_DIRECTORY "${licence_origin}/LICENSES")
    file(COPY "${licence_origin}/LICENSES" DESTINATION "${OUT_DIR}/share/licenses/${package}")
  endif()
  foreach(extra MODIFIED.txt NOTICE NOTICE.txt)
    if(EXISTS "${licence_origin}/${extra}")
      file(COPY "${licence_origin}/${extra}" DESTINATION "${OUT_DIR}/share/licenses/${package}")
    endif()
  endforeach()

  list(APPEND staged "${package}")
endforeach()

if(missing)
  message(FATAL_ERROR
    "dependency payload: ${missing} not found in ${prefix_source}: ${prefixes}.\n"
    "These are required by the installed gnuradio4Config.cmake, so an installer "
    "without them cannot be configured against.")
endif()

# One file naming everything bundled and under what terms, beside the licences.
# This is the enumeration a reader can check: every package GR 4 carries, with
# its licence, in one place.
set(notice "GNU Radio 4 carries the following dependencies.\n\nThese are required to build against GR 4: its installed CMake package\nconfiguration calls find_dependency() on each of them, so a project cannot\nconfigure against GR 4 without them. They are not optional and not separable\nfrom GR 4's development files. They are not GNU Radio's own code, and each\nremains under its own licence.\n\n")
foreach(package IN LISTS staged)
  string(APPEND notice "  ${package} - ${payload_licence_${package}}, see share/licenses/${package}\n")
endforeach()
string(APPEND notice "\n\nLGPL-3.0-or-later notice, required by section 3 of that licence:\n\n  vir-simd is header-only. Its templates and inline functions are compiled\n  into GNU Radio's own binaries, so those binaries incorporate material\n  covered by the GNU Lesser General Public License version 3. The licence\n  texts are in share/licenses/vir-simd, which carries the GPL-3.0 text as\n  well as the LGPL-3.0 one, because the LGPL is the GPL with added\n  permissions. The modifications GNU Radio applies to vir-simd are recorded\n  there too, with their dates.\n")
file(WRITE "${OUT_DIR}/share/licenses/DEPENDENCIES.txt" "${notice}")

message(STATUS "Dependency payload staged: ${staged}")
