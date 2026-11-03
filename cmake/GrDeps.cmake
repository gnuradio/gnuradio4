#[[
Third-party dependencies, built from source by the gr-deps superbuild.

The GNU Radio modules no longer fetch their own dependencies inside their CMake
graph, so something has to put vir-simd, boost.ut, cpp-httplib, SoapySDR and the
rest into the prefix before a module configures. gr-deps is that something: a
second superbuild, driven from here as one more ExternalProject that every
module depends on.

What it builds follows the module selection rather than a fixed list. Each
module declares the gr-deps groups it needs with DEPS_GROUPS in modules.cmake,
and the union over the selected modules becomes gr-deps' TARGET_GR_* arguments,
so a workspace of core and blocks never builds the control plane's Boost.

Test-only dependencies follow GR4_BUILD_TESTING rather than the groups, so a
build with tests off does not build boost.ut or GoogleTest.

    gr-deps          configures and builds the dependency superbuild
    gr-deps-download clones it when no checkout exists yet
    gr-deps-clean    removes its build and stamp state, not the installed files

Set GR4_BUILD_DEPS to OFF to use dependencies already on the machine; nothing
below is then created and the modules are configured exactly as before.
]]

option(GR4_BUILD_DEPS "Build third-party dependencies from source with the gr-deps superbuild" ON)

set(GR4_DEPS_SOURCE_DIR
    ""
    CACHE PATH "Existing gr-deps source tree to build instead of the one below GR4_SOURCE_ROOT")
set(GR4_DEPS_REPOSITORY
    "https://github.com/johnwparent/gnuradio4-superbuild.git"
    CACHE STRING "Git repository used when gr-deps is missing")
set(GR4_DEPS_REF
    "main"
    CACHE STRING "Git branch, tag, or commit used when cloning gr-deps")

# The dependencies share the modules' prefix by default, which is what makes
# them findable and runnable without further wiring: the prefix is already on
# every child's CMAKE_PREFIX_PATH and on PATH in the activation scripts. A
# separate prefix is supported, and then added to both explicitly.
#
# Empty follows CMAKE_INSTALL_PREFIX on every configure instead of caching
# whatever it was first configured with: a cached copy would keep installing
# the dependencies into the old prefix once -DCMAKE_INSTALL_PREFIX moves, and
# the modules would search a prefix nothing writes to any more.
set(GR4_DEPS_INSTALL_PREFIX
    ""
    CACHE PATH "Install prefix for the dependencies; empty follows CMAKE_INSTALL_PREFIX")
if(GR4_DEPS_INSTALL_PREFIX STREQUAL "")
  set(GR4_DEPS_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
endif()

# Empty follows CMAKE_BUILD_TYPE, except that a build type the dependencies do
# not know, such as this workspace's RelWithAssert, becomes Release: a project
# compiled under an unknown build type gets no optimisation flags at all.
set(GR4_DEPS_BUILD_TYPE
    ""
    CACHE STRING "Build type for the dependencies; empty follows CMAKE_BUILD_TYPE")
set(GR4_DEPS_CMAKE_ARGS
    ""
    CACHE STRING "Additional CMake arguments passed only to the gr-deps superbuild")

cmake_path(IS_ABSOLUTE CMAKE_INSTALL_LIBDIR gr4_deps_libdir_is_absolute)
if(gr4_deps_libdir_is_absolute)
  set(GR4_DEPS_LIBDIR_FULL "${CMAKE_INSTALL_LIBDIR}")
else()
  set(GR4_DEPS_LIBDIR_FULL "${GR4_DEPS_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

set(GR4_DEPS_TARGET_NAME "gr-deps")

# The gr-deps group each DEPS_GROUPS value selects.
set(GR4_DEPS_KNOWN_GROUPS core blocks control-plane studio)

function(gr4_deps_group_option output group)
  if(group STREQUAL "core")
    set(option_name TARGET_GR_CORE)
  elseif(group STREQUAL "blocks")
    set(option_name TARGET_GR_BLOCKS)
  elseif(group STREQUAL "control-plane")
    set(option_name TARGET_GR_CONTROL_PLANE)
  elseif(group STREQUAL "studio")
    set(option_name TARGET_GR_STUDIO)
  else()
    message(FATAL_ERROR
      "Unknown gr-deps group '${group}'. Known groups: ${GR4_DEPS_KNOWN_GROUPS}")
  endif()
  set("${output}" "${option_name}" PARENT_SCOPE)
endfunction()

# The build type the dependencies are compiled with, which also decides the
# MSVC runtime gr-deps forces on every subproject.
function(gr4_deps_build_type output)
  if(NOT GR4_DEPS_BUILD_TYPE STREQUAL "")
    set("${output}" "${GR4_DEPS_BUILD_TYPE}" PARENT_SCOPE)
    return()
  endif()
  set(standard_build_types Debug Release RelWithDebInfo MinSizeRel)
  if(CMAKE_BUILD_TYPE IN_LIST standard_build_types)
    set("${output}" "${CMAKE_BUILD_TYPE}" PARENT_SCOPE)
  else()
    set("${output}" "Release" PARENT_SCOPE)
  endif()
endfunction()

function(gr4_add_deps_project)
  set(multi_value_args GROUPS)
  cmake_parse_arguments(DEPS "" "" "${multi_value_args}" ${ARGN})

  set(name "${GR4_DEPS_TARGET_NAME}")
  if(TARGET "${name}")
    message(FATAL_ERROR "The '${name}' project was already created")
  endif()

  foreach(group IN LISTS DEPS_GROUPS)
    if(NOT group IN_LIST GR4_DEPS_KNOWN_GROUPS)
      message(FATAL_ERROR
        "Unknown gr-deps group '${group}'. Known groups: ${GR4_DEPS_KNOWN_GROUPS}")
    endif()
  endforeach()

  # Every group is passed, on or off, so that a workspace that drops a module
  # also stops building what only that module needed. gr-deps re-derives the
  # individual dependency toggles from these on every configure, so a group
  # turned on later still turns on everything it implies.
  string(REPLACE ";" " " groups_line "${DEPS_GROUPS}")
  set(group_args "")
  foreach(group IN LISTS GR4_DEPS_KNOWN_GROUPS)
    gr4_deps_group_option(option_name "${group}")
    if(group IN_LIST DEPS_GROUPS)
      list(APPEND group_args "-D${option_name}:BOOL=ON")
    else()
      list(APPEND group_args "-D${option_name}:BOOL=OFF")
    endif()
  endforeach()

  set(deps_source_dir "${GR4_DEPS_SOURCE_DIR}")
  if(deps_source_dir STREQUAL "")
    cmake_path(
      ABSOLUTE_PATH name
      BASE_DIRECTORY "${GR4_SOURCE_ROOT}"
      NORMALIZE
      OUTPUT_VARIABLE deps_source_dir)
  else()
    cmake_path(
      ABSOLUTE_PATH deps_source_dir BASE_DIRECTORY "${CMAKE_SOURCE_DIR}" NORMALIZE)
    if(NOT EXISTS "${deps_source_dir}/CMakeLists.txt")
      message(FATAL_ERROR
        "GR4_DEPS_SOURCE_DIR is '${deps_source_dir}', but ${deps_source_dir}/CMakeLists.txt does not exist")
    endif()
    message(STATUS "${name}: using sources in ${deps_source_dir}")
  endif()

  # Same rule as the modules: an existing working copy is developer-owned and
  # never touched, a missing one is cloned on the first build.
  if(EXISTS "${deps_source_dir}/CMakeLists.txt")
    set(download_args
        DOWNLOAD_COMMAND ""
        UPDATE_COMMAND ""
        PATCH_COMMAND "")
  else()
    set(download_args
        DOWNLOAD_COMMAND
          "${CMAKE_COMMAND}"
          "-DGIT_EXECUTABLE:FILEPATH=${GIT_EXECUTABLE}"
          "-DSOURCE_DIR:PATH=<SOURCE_DIR>"
          "-DSOURCE_MARKER:STRING=CMakeLists.txt"
          "-DGIT_REPOSITORY:STRING=${GR4_DEPS_REPOSITORY}"
          "-DGIT_TAG:STRING=${GR4_DEPS_REF}"
          -P "${CMAKE_SOURCE_DIR}/cmake/CloneIfMissing.cmake"
        UPDATE_COMMAND ""
        PATCH_COMMAND "")
  endif()

  gr4_deps_build_type(deps_build_type)

  set(deps_binary_dir "${CMAKE_BINARY_DIR}/projects/${name}")
  set(deps_stamp_dir "${CMAKE_BINARY_DIR}/stamp/${name}")

  # gr-deps enables no languages, so its own cache reaches none of the builds
  # it spawns. GR_DEPS_EXTRA_CMAKE_ARGS is handed to each of them verbatim,
  # which is how the install layout this superbuild forces on every module
  # reaches the dependencies: left to GNUInstallDirs, zlib and SoapySDR land in
  # lib64 or lib/<triple> on some distributions, outside the library path the
  # activation script sets.
  #
  # gr-deps expands the value as an ordinary CMake list, so the arguments have
  # to reach it separated by ";". They are joined with "|" here and ";" is what
  # arrives: LIST_SEPARATOR on the ExternalProject below rewrites the one into
  # the other, which is the only way to carry a ";" through CMAKE_ARGS. A value
  # containing "|" would be rewritten into a separator as well, so it is
  # rejected rather than silently split.
  set(forwarded_args
      "-DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}"
      "-DCMAKE_INSTALL_BINDIR=${CMAKE_INSTALL_BINDIR}"
      "-DBUILD_SHARED_LIBS=${BUILD_SHARED_LIBS}")
  foreach(variable CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_TOOLCHAIN_FILE)
    if(DEFINED ${variable} AND NOT "${${variable}}" STREQUAL "")
      list(APPEND forwarded_args "-D${variable}=${${variable}}")
    endif()
  endforeach()
  foreach(argument IN LISTS forwarded_args)
    string(FIND "${argument}" "|" separator_position)
    if(NOT separator_position EQUAL -1)
      message(FATAL_ERROR
        "'${argument}' cannot be forwarded to the dependency builds: it contains "
        "the separator the arguments are carried with")
    endif()
  endforeach()
  list(JOIN forwarded_args "|" forwarded_args_value)

  # The rewrite applies to every CMAKE_ARGS entry, GR4_DEPS_CMAKE_ARGS included,
  # so a "|" supplied there would be split into a separate argument just as
  # silently.
  foreach(argument IN LISTS GR4_DEPS_CMAKE_ARGS)
    string(FIND "${argument}" "|" separator_position)
    if(NOT separator_position EQUAL -1)
      message(FATAL_ERROR
        "GR4_DEPS_CMAKE_ARGS entry '${argument}' cannot be passed to ${name}: it "
        "contains the separator the arguments are carried with")
    endif()
  endforeach()

  set(cmake_args
      "-DCMAKE_INSTALL_PREFIX:PATH=${GR4_DEPS_INSTALL_PREFIX}"
      "-DGR_DEPS_INSTALL_PREFIX:PATH=${GR4_DEPS_INSTALL_PREFIX}"
      "-DGR_DEPS_BUILD_TYPE:STRING=${deps_build_type}"
      "-DCMAKE_BUILD_TYPE:STRING=${deps_build_type}"
      "-DGR_DEPS_EXTRA_CMAKE_ARGS:STRING=${forwarded_args_value}"
      "-DTARGET_TESTS:BOOL=${GR4_BUILD_TESTING}"
      ${group_args}
      ${GR4_DEPS_CMAKE_ARGS})

  # Perl, NASM and OpenSSL are not CMake builds and read none of the above, so
  # the compiler also goes through the environment they do read.
  set(deps_environment "")
  if(DEFINED CMAKE_C_COMPILER AND NOT "${CMAKE_C_COMPILER}" STREQUAL "")
    list(APPEND deps_environment "CC=${CMAKE_C_COMPILER}")
  endif()
  if(DEFINED CMAKE_CXX_COMPILER AND NOT "${CMAKE_CXX_COMPILER}" STREQUAL "")
    list(APPEND deps_environment "CXX=${CMAKE_CXX_COMPILER}")
  endif()
  if(NOT gr4_effective_build_parallel_level STREQUAL "")
    list(APPEND deps_environment
         "CMAKE_BUILD_PARALLEL_LEVEL=${gr4_effective_build_parallel_level}")
  endif()

  # A multi-config generator builds every dependency in its own default
  # configuration unless the configuration is named here, whatever the build
  # type says.
  get_property(is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  set(build_command "${CMAKE_COMMAND}" -E env ${deps_environment}
                    "${CMAKE_COMMAND}" --build "<BINARY_DIR>")
  if(is_multi_config)
    list(APPEND build_command --config "${deps_build_type}")
  endif()

  ExternalProject_Add(
    "${name}"
    ${download_args}
    SOURCE_DIR "${deps_source_dir}"
    BINARY_DIR "${deps_binary_dir}"
    STAMP_DIR "${deps_stamp_dir}"
    INSTALL_DIR "${GR4_DEPS_INSTALL_PREFIX}"
    LIST_SEPARATOR "|"
    CMAKE_GENERATOR "${CMAKE_GENERATOR}"
    CMAKE_ARGS ${cmake_args}
    BUILD_COMMAND ${build_command}
    # each dependency installs itself from gr-deps' build step
    INSTALL_COMMAND ""
    BUILD_ALWAYS TRUE
    STEP_TARGETS download
    EXCLUDE_FROM_ALL TRUE
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE)

  add_custom_target(
    "${name}-clean"
    COMMAND "${CMAKE_COMMAND}" -E rm -rf "${deps_binary_dir}" "${deps_stamp_dir}"
    COMMENT "Removing ${name}'s build state")

  set_property(GLOBAL APPEND PROPERTY GR4_PROJECT_TARGETS "${name}")
  set_property(GLOBAL APPEND PROPERTY GR4_SOURCE_TARGETS "${name}-download")

  message(STATUS "Dependency groups: ${groups_line}")
endfunction()
