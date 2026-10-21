cmake_minimum_required(VERSION 3.27)

project(gr4-super NONE)

include(ExternalProject)
include(cmake/ModuleRegistry.cmake)
find_package(Git REQUIRED)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX
      "${CMAKE_SOURCE_DIR}/install"
      CACHE PATH "Shared install prefix for all GNU Radio 4 projects" FORCE)
endif()

set(GR4_SOURCE_ROOT
    "${CMAKE_SOURCE_DIR}/src"
    CACHE PATH "Directory containing the GNU Radio 4 source repositories")
set(GR4_EXTRA_PROJECTS
    ""
    CACHE STRING "Semicolon-separated paths to additional CMake projects")
set(GR4_EXTRA_CMAKE_ARGS
    ""
    CACHE STRING "Additional CMake arguments passed to every project")
set(GR4_MODULE_GROUPS
    "base"
    CACHE STRING "Semicolon-separated module groups to build")
set(GR4_MODULES
    ""
    CACHE STRING "Additional registered modules to build")
set(GR4_EXCLUDE_MODULES
    ""
    CACHE STRING "Registered modules to exclude from the selected groups")
set(GR4_BUILD_PARALLEL_LEVEL
    ""
    CACHE STRING "Maximum parallel jobs for child builds; defaults to CMAKE_BUILD_PARALLEL_LEVEL from the environment")
set(CMAKE_INSTALL_LIBDIR
    "lib"
    CACHE STRING "Relative library installation directory shared by all child projects")
set(CMAKE_INSTALL_BINDIR
    "bin"
    CACHE STRING "Relative binary installation directory shared by all child projects")

set(gr4_effective_build_parallel_level "${GR4_BUILD_PARALLEL_LEVEL}")
if(gr4_effective_build_parallel_level STREQUAL ""
   AND DEFINED ENV{CMAKE_BUILD_PARALLEL_LEVEL}
   AND NOT "$ENV{CMAKE_BUILD_PARALLEL_LEVEL}" STREQUAL "")
  set(gr4_effective_build_parallel_level "$ENV{CMAKE_BUILD_PARALLEL_LEVEL}")
endif()
if(NOT gr4_effective_build_parallel_level STREQUAL ""
   AND NOT gr4_effective_build_parallel_level MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR
    "The build parallel level must be a positive integer, got '${gr4_effective_build_parallel_level}'")
endif()

option(GR4_BUILD_TESTING "Build tests in the child projects" ON)
option(GR4_FETCH_DEPS "Allow child projects to fetch missing dependencies" ON)
option(GR4_WARNINGS_AS_ERRORS "Build child projects with warnings as errors" ON)
option(GR4_ADDRESS_SANITIZER "Build child projects with AddressSanitizer" OFF)
option(GR4_UB_SANITIZER "Build child projects with UndefinedBehaviorSanitizer" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Deprecated compatibility switches. ON still includes the corresponding
# modules; OFF and empty defer to module selection so stale preset caches do
# not override GR4_MODULE_GROUPS.
foreach(legacy_option
    GR4_ENABLE_INCUBATOR
    GR4_ENABLE_CONTROL_PLANE
    GR4_ENABLE_STUDIO)
  set(
    "${legacy_option}"
    ""
    CACHE STRING "Deprecated enable override; use GR4_MODULE_GROUPS or GR4_MODULES")
  set_property(CACHE "${legacy_option}" PROPERTY STRINGS "" ON OFF)
endforeach()

cmake_path(IS_ABSOLUTE CMAKE_INSTALL_LIBDIR install_libdir_is_absolute)
if(install_libdir_is_absolute)
  set(GR4_INSTALL_LIBDIR_FULL "${CMAKE_INSTALL_LIBDIR}")
else()
  set(GR4_INSTALL_LIBDIR_FULL "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()

set_property(GLOBAL PROPERTY GR4_PROJECT_TARGETS "")
set_property(GLOBAL PROPERTY GR4_SOURCE_TARGETS "")
set_property(GLOBAL PROPERTY GR4_CTEST_DIRS "")
set_property(GLOBAL PROPERTY GR4_TEST_TARGETS "")

function(gr4_add_project name)
  set(options TESTS)
  set(one_value_args SOURCE_DIR SOURCE_SUBDIR GIT_REPOSITORY GIT_TAG)
  set(multi_value_args DEPENDS CMAKE_ARGS)
  cmake_parse_arguments(PROJECT "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(NOT name MATCHES "^[A-Za-z0-9][A-Za-z0-9_.+-]*$")
    message(FATAL_ERROR "Invalid project name '${name}'")
  endif()
  if(TARGET "${name}")
    message(FATAL_ERROR "Duplicate project name '${name}'")
  endif()
  if(NOT PROJECT_SOURCE_DIR)
    message(FATAL_ERROR "gr4_add_project(${name}) requires SOURCE_DIR")
  endif()

  cmake_path(ABSOLUTE_PATH PROJECT_SOURCE_DIR BASE_DIRECTORY "${CMAKE_SOURCE_DIR}" NORMALIZE)
  set(project_binary_dir "${CMAKE_BINARY_DIR}/projects/${name}")
  set(project_stamp_dir "${CMAKE_BINARY_DIR}/stamp/${name}")
  set(project_cmake_file "${PROJECT_SOURCE_DIR}/CMakeLists.txt")
  set(project_source_marker "CMakeLists.txt")
  if(PROJECT_SOURCE_SUBDIR)
    set(project_cmake_file "${PROJECT_SOURCE_DIR}/${PROJECT_SOURCE_SUBDIR}/CMakeLists.txt")
    set(project_source_marker "${PROJECT_SOURCE_SUBDIR}/CMakeLists.txt")
  endif()

  # Existing working copies are entirely developer-owned. Missing first-party
  # sources are cloned by ExternalProject on the first build.
  if(EXISTS "${project_cmake_file}")
    set(download_args
        DOWNLOAD_COMMAND ""
        UPDATE_COMMAND ""
        PATCH_COMMAND "")
  elseif(PROJECT_GIT_REPOSITORY)
    if(NOT PROJECT_GIT_TAG)
      set(PROJECT_GIT_TAG main)
    endif()
    set(download_args
        DOWNLOAD_COMMAND
          "${CMAKE_COMMAND}"
          "-DGIT_EXECUTABLE:FILEPATH=${GIT_EXECUTABLE}"
          "-DSOURCE_DIR:PATH=<SOURCE_DIR>"
          "-DSOURCE_MARKER:STRING=${project_source_marker}"
          "-DGIT_REPOSITORY:STRING=${PROJECT_GIT_REPOSITORY}"
          "-DGIT_TAG:STRING=${PROJECT_GIT_TAG}"
          -P "${CMAKE_SOURCE_DIR}/cmake/CloneIfMissing.cmake"
        UPDATE_COMMAND ""
        PATCH_COMMAND "")
  else()
    message(FATAL_ERROR
      "Missing ${project_cmake_file} for '${name}' and no GIT_REPOSITORY was provided")
  endif()

  set(prefix_path "${CMAKE_INSTALL_PREFIX}")
  if(CMAKE_PREFIX_PATH)
    list(APPEND prefix_path ${CMAKE_PREFIX_PATH})
  endif()
  list(JOIN prefix_path "|" prefix_path_arg)

  set(child_testing OFF)
  if(PROJECT_TESTS AND GR4_BUILD_TESTING)
    set(child_testing ON)
  endif()

  set(cmake_args
      "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
      "-DCMAKE_INSTALL_LIBDIR:STRING=${CMAKE_INSTALL_LIBDIR}"
      "-DCMAKE_PREFIX_PATH:STRING=${prefix_path_arg}"
      "-DCMAKE_PROGRAM_PATH:PATH=${CMAKE_INSTALL_PREFIX}/bin"
      "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
      "-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}"
      "-DENABLE_TESTING:BOOL=${child_testing}"
      "-DPKG_CONFIG_USE_CMAKE_PREFIX_PATH:BOOL=ON"
      "-DGR_USE_FETCHCONTENT_DEPS:BOOL=${GR4_FETCH_DEPS}"
      "-DWARNINGS_AS_ERRORS:BOOL=${GR4_WARNINGS_AS_ERRORS}"
      "-DADDRESS_SANITIZER:BOOL=${GR4_ADDRESS_SANITIZER}"
      "-DUB_SANITIZER:BOOL=${GR4_UB_SANITIZER}")

  foreach(variable CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_TOOLCHAIN_FILE)
    if(DEFINED ${variable} AND NOT "${${variable}}" STREQUAL "")
      list(APPEND cmake_args "-D${variable}:FILEPATH=${${variable}}")
    endif()
  endforeach()
  list(APPEND cmake_args ${GR4_EXTRA_CMAKE_ARGS} ${PROJECT_CMAKE_ARGS})

  set(build_command_args "")
  if(NOT gr4_effective_build_parallel_level STREQUAL "")
    set(build_command_args
        BUILD_COMMAND
          "${CMAKE_COMMAND}" -E env
          "CMAKE_BUILD_PARALLEL_LEVEL=${gr4_effective_build_parallel_level}"
          "${CMAKE_COMMAND}" --build "<BINARY_DIR>")
    if(name STREQUAL "gnuradio4-core")
      list(APPEND cmake_args
        "-DGR_BUILD_PARALLEL_LEVEL:STRING=${gr4_effective_build_parallel_level}")
    endif()
  endif()

  set(source_subdir_args "")
  if(PROJECT_SOURCE_SUBDIR)
    set(source_subdir_args SOURCE_SUBDIR "${PROJECT_SOURCE_SUBDIR}")
  endif()

  ExternalProject_Add(
    "${name}"
    ${download_args}
    SOURCE_DIR "${PROJECT_SOURCE_DIR}"
    ${source_subdir_args}
    BINARY_DIR "${project_binary_dir}"
    STAMP_DIR "${project_stamp_dir}"
    INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
    LIST_SEPARATOR "|"
    CMAKE_GENERATOR "${CMAKE_GENERATOR}"
    CMAKE_ARGS ${cmake_args}
    ${build_command_args}
    BUILD_ALWAYS TRUE
    STEP_TARGETS download
    DEPENDS ${PROJECT_DEPENDS}
    EXCLUDE_FROM_ALL TRUE
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE
    USES_TERMINAL_INSTALL TRUE)

  add_custom_target(
    "${name}-clean"
    COMMAND "${CMAKE_COMMAND}" -E rm -rf "${project_binary_dir}" "${project_stamp_dir}"
    COMMENT "Removing ${name}'s build state")

  set_property(GLOBAL APPEND PROPERTY GR4_PROJECT_TARGETS "${name}")
  set_property(GLOBAL APPEND PROPERTY GR4_SOURCE_TARGETS "${name}-download")
  if(child_testing)
    set_property(GLOBAL APPEND PROPERTY GR4_CTEST_DIRS "${project_binary_dir}")
    set_property(GLOBAL APPEND PROPERTY GR4_TEST_TARGETS "${name}")
  endif()
endfunction()

function(gr4_add_node_project name)
  set(options TESTS)
  set(one_value_args SOURCE_DIR SOURCE_PROVIDER GIT_REPOSITORY GIT_TAG)
  set(multi_value_args DEPENDS)
  cmake_parse_arguments(PROJECT "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(NOT PROJECT_SOURCE_DIR)
    message(FATAL_ERROR "gr4_add_node_project(${name}) requires SOURCE_DIR")
  endif()
  if(TARGET "${name}")
    message(FATAL_ERROR "Duplicate project name '${name}'")
  endif()

  find_program(NPM_EXECUTABLE npm REQUIRED)
  cmake_path(ABSOLUTE_PATH PROJECT_SOURCE_DIR BASE_DIRECTORY "${CMAKE_SOURCE_DIR}" NORMALIZE)

  if(EXISTS "${PROJECT_SOURCE_DIR}/package.json")
    set(download_args
        DOWNLOAD_COMMAND ""
        UPDATE_COMMAND ""
        PATCH_COMMAND "")
  elseif(PROJECT_GIT_REPOSITORY)
    if(NOT PROJECT_GIT_TAG)
      set(PROJECT_GIT_TAG main)
    endif()
    set(download_args
        DOWNLOAD_COMMAND
          "${CMAKE_COMMAND}"
          "-DGIT_EXECUTABLE:FILEPATH=${GIT_EXECUTABLE}"
          "-DSOURCE_DIR:PATH=<SOURCE_DIR>"
          "-DSOURCE_MARKER:STRING=package.json"
          "-DGIT_REPOSITORY:STRING=${PROJECT_GIT_REPOSITORY}"
          "-DGIT_TAG:STRING=${PROJECT_GIT_TAG}"
          -P "${CMAKE_SOURCE_DIR}/cmake/CloneIfMissing.cmake"
        UPDATE_COMMAND ""
        PATCH_COMMAND "")
  elseif(PROJECT_SOURCE_PROVIDER)
    # The provider owns and populates a repository shared with this project.
    set(download_args
        DOWNLOAD_COMMAND "${CMAKE_COMMAND}" -E true
        UPDATE_COMMAND ""
        PATCH_COMMAND "")
  else()
    message(FATAL_ERROR
      "Missing ${PROJECT_SOURCE_DIR}/package.json for '${name}'; provide "
      "GIT_REPOSITORY or SOURCE_PROVIDER")
  endif()

  set(node_stamp_dir "${CMAKE_BINARY_DIR}/stamp/${name}")
  set(node_environment
      "${CMAKE_COMMAND}" -E env
      "GR4_PREFIX=${CMAKE_INSTALL_PREFIX}"
      "GR4_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}"
      "PATH=${CMAKE_INSTALL_PREFIX}/bin:$ENV{PATH}"
      "CMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}"
      "PKG_CONFIG_PATH=${GR4_INSTALL_LIBDIR_FULL}/pkgconfig:${CMAKE_INSTALL_PREFIX}/share/pkgconfig")

  ExternalProject_Add(
    "${name}"
    ${download_args}
    SOURCE_DIR "${PROJECT_SOURCE_DIR}"
    STAMP_DIR "${node_stamp_dir}"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND
      "${CMAKE_COMMAND}"
      "-DNPM_EXECUTABLE:FILEPATH=${NPM_EXECUTABLE}"
      "-DSOURCE_DIR:PATH=<SOURCE_DIR>"
      "-DSTAMP_DIR:PATH=${node_stamp_dir}"
      -P "${CMAKE_SOURCE_DIR}/cmake/EnsureNodeDependencies.cmake"
      COMMAND
      ${node_environment}
      "${NPM_EXECUTABLE}" --prefix "<SOURCE_DIR>" run build
    INSTALL_COMMAND ""
    BUILD_IN_SOURCE TRUE
    BUILD_ALWAYS TRUE
    STEP_TARGETS download
    DEPENDS ${PROJECT_DEPENDS}
    EXCLUDE_FROM_ALL TRUE
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE)

  add_custom_target(
    "${name}-clean"
    COMMAND "${CMAKE_COMMAND}" -E rm -rf "${node_stamp_dir}"
    COMMENT "Removing ${name}'s superbuild state")

  set_property(GLOBAL APPEND PROPERTY GR4_PROJECT_TARGETS "${name}")
  set_property(GLOBAL APPEND PROPERTY GR4_SOURCE_TARGETS "${name}-download")

  if(PROJECT_TESTS AND GR4_BUILD_TESTING)
    add_custom_target(
      "${name}-check"
      COMMAND
        ${node_environment}
        "${NPM_EXECUTABLE}" --prefix "${PROJECT_SOURCE_DIR}" run lint
      COMMAND
        ${node_environment}
        "${NPM_EXECUTABLE}" --prefix "${PROJECT_SOURCE_DIR}" test
      DEPENDS "${name}"
      USES_TERMINAL
      COMMENT "Linting and testing ${name}")
    set_property(GLOBAL APPEND PROPERTY GR4_TEST_TARGETS "${name}-check")
  endif()
endfunction()

include("${CMAKE_SOURCE_DIR}/modules.cmake")

# Local manifests use the same registration API as the checked-in manifest.
# The file is ignored by git.
include("${CMAKE_SOURCE_DIR}/projects.local.cmake" OPTIONAL)

# For the common one-off case, paths may also be supplied directly. These
# modules retain the historical dependency on the installed blocks SDK.
foreach(source_dir IN LISTS GR4_EXTRA_PROJECTS)
  cmake_path(ABSOLUTE_PATH source_dir BASE_DIRECTORY "${CMAKE_SOURCE_DIR}" NORMALIZE)
  cmake_path(GET source_dir FILENAME project_name)
  gr4_register_module(
    NAME "${project_name}"
    TYPE CMAKE
    SOURCE_DIR "${source_dir}"
    DEPENDS gnuradio4-blocks)
  list(APPEND GR4_MODULES "${project_name}")
endforeach()

gr4_realize_modules()

get_property(project_targets GLOBAL PROPERTY GR4_PROJECT_TARGETS)
get_property(source_targets GLOBAL PROPERTY GR4_SOURCE_TARGETS)
get_property(ctest_dirs GLOBAL PROPERTY GR4_CTEST_DIRS)
get_property(test_targets GLOBAL PROPERTY GR4_TEST_TARGETS)

add_custom_target(gr4 ALL DEPENDS ${project_targets})
add_custom_target(sources DEPENDS ${source_targets})

set(integration_test_targets "")
if(UNIX)
  add_custom_target(
    activate-smoke
    COMMAND /bin/sh "${CMAKE_SOURCE_DIR}/tests/activate-smoke.sh"
    USES_TERMINAL
    COMMENT "Sourcing the activation script in every available shell")
  list(APPEND integration_test_targets activate-smoke)
endif()

if("gnuradio4-blocks" IN_LIST GR4_SELECTED_MODULES)
  set(smoke_binary_dir "${CMAKE_BINARY_DIR}/tests/oot-smoke")
  set(smoke_cmake_args
      "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
      "-DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}")
  foreach(variable CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_TOOLCHAIN_FILE)
    if(DEFINED ${variable} AND NOT "${${variable}}" STREQUAL "")
      list(APPEND smoke_cmake_args "-D${variable}:FILEPATH=${${variable}}")
    endif()
  endforeach()

  add_custom_target(
    oot-smoke
    COMMAND
      "${CMAKE_COMMAND}"
      -S "${CMAKE_SOURCE_DIR}/tests/oot-smoke"
      -B "${smoke_binary_dir}"
      -G "${CMAKE_GENERATOR}"
      ${smoke_cmake_args}
    COMMAND "${CMAKE_COMMAND}" --build "${smoke_binary_dir}"
    COMMAND
      "${CMAKE_CTEST_COMMAND}"
      --test-dir "${smoke_binary_dir}"
      --output-on-failure
      --no-tests=error
    DEPENDS gnuradio4-blocks
    USES_TERMINAL
    COMMENT "Building and testing an out-of-tree consumer against the installed GNU Radio 4 SDK")
  list(APPEND integration_test_targets oot-smoke)
endif()

if("gnuradio4-control-plane" IN_LIST GR4_SELECTED_MODULES AND UNIX)
  add_custom_target(
    control-plane-smoke
    COMMAND
      "${CMAKE_COMMAND}" -E env
      "GNURADIO4_PLUGIN_DIRECTORIES=${GR4_INSTALL_LIBDIR_FULL}/gnuradio-4/plugins:${GR4_INSTALL_LIBDIR_FULL}"
      "LD_LIBRARY_PATH=${GR4_INSTALL_LIBDIR_FULL}"
      "DYLD_LIBRARY_PATH=${GR4_INSTALL_LIBDIR_FULL}"
      /bin/sh "${CMAKE_SOURCE_DIR}/tests/control-plane-smoke.sh"
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/gr4cp_server"
      "${CMAKE_COMMAND}"
    DEPENDS gnuradio4-control-plane
    USES_TERMINAL
    COMMENT "Verifying the installed control plane and GNU Radio 4 plugin catalog")
  list(APPEND integration_test_targets control-plane-smoke)
endif()

set(check_commands "")
set(check_environment
    "${CMAKE_COMMAND}" -E env
    "GNURADIO4_PLUGIN_DIRECTORIES=${GR4_INSTALL_LIBDIR_FULL}/gnuradio-4/plugins:${GR4_INSTALL_LIBDIR_FULL}"
    "LD_LIBRARY_PATH=${GR4_INSTALL_LIBDIR_FULL}"
    "DYLD_LIBRARY_PATH=${GR4_INSTALL_LIBDIR_FULL}")
foreach(test_dir IN LISTS ctest_dirs)
  list(
    APPEND
    check_commands
    COMMAND
      ${check_environment}
      "${CMAKE_CTEST_COMMAND}"
      --test-dir "${test_dir}"
      --output-on-failure
      --no-tests=error)
endforeach()

if(check_commands)
  add_custom_target(
    check
    ${check_commands}
    DEPENDS ${integration_test_targets} ${test_targets}
    USES_TERMINAL
    COMMENT "Running GNU Radio 4 integration tests")
else()
  add_custom_target(
    check
    DEPENDS ${integration_test_targets} ${test_targets})
endif()

configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/activate.sh.in"
  "${CMAKE_BINARY_DIR}/activate.sh"
  @ONLY)

message(STATUS "GNU Radio 4 sources: ${GR4_SOURCE_ROOT}")
message(STATUS "Shared install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Modules: ${GR4_SELECTED_MODULES}")
message(STATUS "After building: source ${CMAKE_BINARY_DIR}/activate.sh")
