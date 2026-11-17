#[[
Unified installer for the superbuild, on Windows and macOS.

Every CMake module already knows how to package itself through CPack (see
gr-core's GrInnoSetup.cmake and GrProductBuild.cmake). This file does not
repackage their files: it asks each module for its own artefacts and composes
them into one installer.

    <name>-package   builds one module's artefacts and copies them, with a small
                     description of them, into ${GR4_PACKAGING_DIR}/components
    gr4-installer    builds every <name>-package and composes the bundle

What "artefacts" means is the one real difference between the two platforms:

    Windows   one self-contained Inno Setup installer per module. The bundle
              embeds them and runs the selected ones silently, one after
              another, and its [Types] come from the module GROUPS.
    macOS     one flat component package per CPack component. A productbuild
              distribution package cannot be nested inside another, so the
              bundle references real component packages and lets macOS
              Installer do the selecting. macOS has no install-type concept, so
              GROUPS decide which choices start ticked instead.

Selecting components does not enforce module dependencies on either platform:
installing blocks without core is allowed.
#]]

set(GR4_PACKAGING_DIR "${CMAKE_BINARY_DIR}/packaging")
set(GR4_PACKAGING_SCRIPT_DIR "${CMAKE_CURRENT_LIST_DIR}")

set(GR4_BUNDLE_VERSION
    "4.0.0"
    CACHE STRING "Version of the unified GNU Radio 4 installer")
# the leading unbalanced { is Inno Setup's escape for a literal brace. Never
# reuse a module installer's AppId here and never change this between releases.
set(GR4_BUNDLE_APP_ID
    "{{6E7C9F41-2B8D-4A55-9C31-0D5E8A7F42B6}"
    CACHE STRING "Inno Setup AppId of the unified GNU Radio 4 installer")
mark_as_advanced(GR4_BUNDLE_APP_ID)

# Debug binaries come from a second superbuild tree, built with
# CMAKE_BUILD_TYPE=Debug: one tree cannot hold two configurations, and the
# presets already give each its own binary dir and install prefix.
set(GR4_DEBUG_PACKAGING_DIR
    ""
    CACHE PATH "packaging/components of a Debug superbuild tree; empty leaves the debug checkboxes out")

set(GR4_BUNDLE_ICON_SOURCE
    ""
    CACHE FILEPATH "PNG the installer icon is generated from; empty uses the logo shipped with the core module")

# never reuse a module installer's identifier here and never change it between
# releases: pkgutil identifies the product by it
set(GR4_BUNDLE_IDENTIFIER
    "org.gnuradio.gr4.bundle"
    CACHE STRING "pkgutil identifier of the unified GNU Radio 4 installer")
mark_as_advanced(GR4_BUNDLE_IDENTIFIER)
set(GR4_MACOS_MIN_VERSION
    "13.0"
    CACHE STRING "Minimum macOS version the unified installer allows")
set(GR4_MACOS_HOST_ARCHITECTURES
    "arm64,x86_64"
    CACHE STRING "Architectures the unified macOS installer may run on")
set(GR4_MACOS_NOTARIZE_PROFILE
    ""
    CACHE STRING "notarytool keychain profile used to notarise the macOS installer; empty skips notarisation")
# Also declared by gr-core's GrProductBuild.cmake for the component packages;
# the signature Gatekeeper actually checks is the one on the distribution
# package this file builds around them.
set(GR4_MACOS_SIGN_IDENTITY
    ""
    CACHE STRING "Developer ID Installer identity for the macOS installer; empty leaves it unsigned")

set(GR4_PACKAGING_ENABLED FALSE)
if(GR_ENABLE_PACKAGING)
  if(WIN32)
    set(GR4_PACKAGING_ENABLED TRUE)
    set(GR4_PACKAGING_FLAVOUR windows)
    set(program_files_x86 "ProgramFiles(x86)")
    find_program(
      GR4_ISCC_EXECUTABLE ISCC.exe
      PATHS "$ENV{${program_files_x86}}/Inno Setup 6" "$ENV{ProgramFiles}/Inno Setup 6"
      DOC "Inno Setup command-line compiler"
      REQUIRED)
  elseif(APPLE)
    set(GR4_PACKAGING_ENABLED TRUE)
    set(GR4_PACKAGING_FLAVOUR macos)
    # part of the Xcode command line tools, which a macOS build already needs
    find_program(GR4_PRODUCTBUILD_EXECUTABLE productbuild DOC "macOS distribution package builder" REQUIRED)
    if(GR4_MACOS_NOTARIZE_PROFILE)
      find_program(GR4_XCRUN_EXECUTABLE xcrun DOC "Xcode command line tool driver" REQUIRED)
    endif()
  else()
    message(STATUS "GR_ENABLE_PACKAGING: the unified installer is only built on Windows and macOS")
  endif()
endif()

# project(... NONE) never sets MSVC, and /DEBUG would break a MinGW build
set(gr4_msvc_toolchain FALSE)
if(WIN32 AND (NOT CMAKE_CXX_COMPILER OR CMAKE_CXX_COMPILER MATCHES "cl(\.exe)?$"))
  set(gr4_msvc_toolchain TRUE)
endif()

set_property(GLOBAL PROPERTY GR4_PACKAGE_TARGETS "")

# Adds <name>-package for a CMake module. Call right after its ExternalProject_Add.
function(gr4_package_project name binary_dir)
  if(NOT GR4_PACKAGING_ENABLED)
    return()
  endif()

  # the two collectors take the same arguments and write the same kind of
  # fragment, so only the script name differs
  if(GR4_PACKAGING_FLAVOUR STREQUAL "macos")
    set(collect_script "CollectComponentPackage.cmake")
  else()
    set(collect_script "CollectComponentInstaller.cmake")
  endif()

  ExternalProject_Add_Step(
    "${name}" package
    COMMENT "Building the ${name} installer"
    COMMAND
      "${CMAKE_COMMAND}"
      "-DNAME:STRING=${name}"
      "-DBINARY_DIR:PATH=${binary_dir}"
      "-DOUT_DIR:PATH=${GR4_PACKAGING_DIR}/components"
      "-DBUILD_PARALLEL_LEVEL:STRING=${gr4_effective_build_parallel_level}"
      -P "${GR4_PACKAGING_SCRIPT_DIR}/${collect_script}"
    DEPENDEES install
    ALWAYS TRUE
    EXCLUDE_FROM_MAIN TRUE
    USES_TERMINAL TRUE)
  ExternalProject_Add_StepTargets("${name}" package)

  set_property(GLOBAL APPEND PROPERTY GR4_PACKAGE_TARGETS "${name}-package")
endfunction()

# Debug information roughly doubles what a packaging build writes to disc, and
# the symbols component is the only thing that consumes it, so it is worth
# being able to turn off for a test installer.
option(GR4_PACKAGE_SYMBOLS "Build debug symbols for the installer's symbols component" ON)

# CMake arguments every child needs when it is going to be packaged. Appended
# by gr4_add_project; empty when packaging is off, so ordinary builds are
# unaffected.
function(gr4_packaging_cmake_args output)
  set(arguments "")
  if(GR4_PACKAGING_ENABLED AND WIN32)
    # release and debug libraries share one bin/, so the debug ones need
    # distinct names: gnuradio-core.dll beside gnuradio-cored.dll
    list(APPEND arguments "-DCMAKE_DEBUG_POSTFIX:STRING=d")
    if(gr4_msvc_toolchain AND GR4_PACKAGE_SYMBOLS)
      # an optimised build emits no PDB by default, which would leave the
      # symbols component with nothing to ship
      list(APPEND arguments
           "-DCMAKE_MSVC_DEBUG_INFORMATION_FORMAT:STRING=ProgramDatabase"
           "-DCMAKE_SHARED_LINKER_FLAGS:STRING=/DEBUG"
           "-DCMAKE_EXE_LINKER_FLAGS:STRING=/DEBUG")
    endif()
  endif()
  set("${output}" "${arguments}" PARENT_SCOPE)
endfunction()

# Writes whatever the bundle needs at configure time and adds gr4-installer.
# Call once, after gr4_realize_modules().
function(gr4_add_installer_target)
  if(NOT GR4_PACKAGING_ENABLED)
    return()
  endif()

  get_property(package_targets GLOBAL PROPERTY GR4_PACKAGE_TARGETS)
  set(modules "")
  foreach(name IN LISTS GR4_SELECTED_MODULES)
    if("${name}-package" IN_LIST package_targets)
      list(APPEND modules "${name}")
    endif()
  endforeach()
  if(NOT modules)
    message(WARNING "GR_ENABLE_PACKAGING is ON but no selected module can build an installer")
    return()
  endif()

  if(GR4_PACKAGING_FLAVOUR STREQUAL "macos")
    _gr4_add_macos_installer_target("${modules}" "${package_targets}")
  else()
    _gr4_add_windows_installer_target("${modules}" "${package_targets}")
  endif()

  if(NOT TARGET package)
    add_custom_target(package DEPENDS gr4-installer)
  endif()
endfunction()

# The macOS bundle. Unlike the Windows one it configures none of gr-core's
# installer templates and registers nothing in the manifest: there is no install
# root to negotiate, and every component package already carries the scripts
# that record it.
function(_gr4_add_macos_installer_target modules package_targets)
  # macOS Installer has no [Types] equivalent, so the module groups are written
  # out for the generator to turn into per-choice start_selected defaults.
  set(groups_content "# Generated by gr-super cmake/PackageGr4.cmake, do not edit.\n")
  foreach(name IN LISTS modules)
    _gr4_get_module_property(module_groups "${name}" GROUPS)
    string(MAKE_C_IDENTIFIER "${name}" module_id)
    string(APPEND groups_content "set(GR4_MODULE_GROUPS_${module_id} [==[${module_groups}]==])\n")
  endforeach()
  file(WRITE "${GR4_PACKAGING_DIR}/gr4_bundle_groups.cmake" "${groups_content}")

  # productbuild refuses a --resources directory that does not exist, and the
  # generator only populates it at build time
  file(MAKE_DIRECTORY "${GR4_PACKAGING_DIR}/resources")

  set(productbuild_arguments
      --distribution "${GR4_PACKAGING_DIR}/gr4_distribution.xml"
      --package-path "${GR4_PACKAGING_DIR}/components"
      --resources "${GR4_PACKAGING_DIR}/resources")
  if(GR4_MACOS_SIGN_IDENTITY)
    list(APPEND productbuild_arguments --sign "${GR4_MACOS_SIGN_IDENTITY}")
  endif()
  set(bundle_output "${CMAKE_BINARY_DIR}/gnuradio4-${GR4_BUNDLE_VERSION}-macos.pkg")

  # Notarisation is opt-in and entirely absent when no profile is configured:
  # the bundle still installs unsigned, after a Gatekeeper right-click.
  set(notarize_commands "")
  if(GR4_MACOS_NOTARIZE_PROFILE)
    set(notarize_commands
        COMMAND "${GR4_XCRUN_EXECUTABLE}" notarytool submit "${bundle_output}" --keychain-profile
                "${GR4_MACOS_NOTARIZE_PROFILE}" --wait
        COMMAND "${GR4_XCRUN_EXECUTABLE}" stapler staple "${bundle_output}")
  endif()

  # module names cannot contain ',' so it is a safe separator on any shell
  list(JOIN modules "," modules_argument)
  add_custom_target(
    gr4-installer
    COMMAND
      "${CMAKE_COMMAND}"
      "-DPACKAGING_DIR:PATH=${GR4_PACKAGING_DIR}"
      "-DGNURADIO4_CMAKE_DIR:PATH=${GR4_INSTALL_LIBDIR_FULL}/cmake/gnuradio4"
      "-DMODULES:STRING=${modules_argument}"
      "-DBUNDLE_VERSION:STRING=${GR4_BUNDLE_VERSION}"
      "-DMIN_OS_VERSION:STRING=${GR4_MACOS_MIN_VERSION}"
      "-DHOST_ARCHITECTURES:STRING=${GR4_MACOS_HOST_ARCHITECTURES}"
      "-DLICENSE_FILE:FILEPATH=${CMAKE_SOURCE_DIR}/LICENSE"
      "-DTEMPLATE:FILEPATH=${GR4_PACKAGING_SCRIPT_DIR}/gr4_distribution.xml.in"
      -P "${GR4_PACKAGING_SCRIPT_DIR}/GenerateBundleDistribution.cmake"
    COMMAND "${GR4_PRODUCTBUILD_EXECUTABLE}" ${productbuild_arguments} "${bundle_output}"
    ${notarize_commands}
    DEPENDS ${package_targets}
    USES_TERMINAL
    VERBATIM
    COMMENT "Composing the unified GNU Radio 4 installer")
endfunction()

# The Windows bundle.
function(_gr4_add_windows_installer_target modules package_targets)
  # Install types are curated layouts rather than raw group names: a user
  # picking "Typical" should not have to know what a module group is. Groups
  # still decide membership, so modules.cmake stays the single source of truth.
  set(type_ids typical complete runtime)
  set(type_description_typical "Typical installation")
  set(type_description_complete "Complete installation")
  set(type_description_runtime "Runtime only")

  # every group that is not already covered by typical or complete
  set(groups "")
  foreach(name IN LISTS modules)
    _gr4_get_module_property(module_groups "${name}" GROUPS)
    foreach(group IN LISTS module_groups)
      if(NOT group IN_LIST groups AND NOT group MATCHES "^(base|full|custom)$")
        list(APPEND groups "${group}")
      endif()
    endforeach()
  endforeach()

  foreach(group IN LISTS groups)
    string(MAKE_C_IDENTIFIER "${group}" group_id)
    string(SUBSTRING "${group}" 0 1 first_letter)
    string(SUBSTRING "${group}" 1 -1 other_letters)
    string(TOUPPER "${first_letter}" first_letter)
    list(APPEND type_ids "${group_id}")
    set(type_description_${group_id} "${first_letter}${other_letters}")
  endforeach()
  list(APPEND type_ids custom)
  set(type_description_custom "Custom installation")

  # which modules each group-derived type installs: the group plus what it
  # needs, which is what the same name means in GR4_MODULE_GROUPS
  set(closure_groups ${groups} base)
  foreach(group IN LISTS closure_groups)
    set(members_${group} "")
    foreach(name IN LISTS modules)
      _gr4_get_module_property(module_groups "${name}" GROUPS)
      if(group IN_LIST module_groups)
        list(APPEND members_${group} "${name}")
      endif()
    endforeach()
    set(members_changed TRUE)
    while(members_changed)
      set(members_changed FALSE)
      foreach(name IN LISTS members_${group})
        _gr4_get_module_property(module_dependencies "${name}" DEPENDS)
        foreach(dependency IN LISTS module_dependencies)
          if(dependency IN_LIST modules AND NOT dependency IN_LIST members_${group})
            list(APPEND members_${group} "${dependency}")
            set(members_changed TRUE)
          endif()
        endforeach()
      endforeach()
    endwhile()
  endforeach()

  set(types_content "# Generated by gr-super cmake/PackageGr4.cmake, do not edit.
")
  string(APPEND types_content "set(GR4_BUNDLE_TYPE_IDS [==[${type_ids}]==])
")
  foreach(type_id IN LISTS type_ids)
    string(APPEND types_content
      "set(GR4_BUNDLE_TYPE_DESCRIPTION_${type_id} [==[${type_description_${type_id}}]==])
")
  endforeach()

  # Per module: the types that install it, the types that also take its
  # development files, and the types that take its debug symbols. Debug
  # binaries belong to no preset at all; they are always an explicit choice.
  foreach(name IN LISTS modules)
    set(module_types complete runtime)
    if(name IN_LIST members_base)
      list(APPEND module_types typical)
    endif()
    foreach(group IN LISTS groups)
      if(name IN_LIST members_${group})
        string(MAKE_C_IDENTIFIER "${group}" group_id)
        list(APPEND module_types "${group_id}")
      endif()
    endforeach()
    list(APPEND module_types custom)

    # a runtime-only install is exactly the one that leaves headers out
    set(develop_types ${module_types})
    list(REMOVE_ITEM develop_types runtime)

    string(MAKE_C_IDENTIFIER "${name}" module_id)
    string(APPEND types_content
      "set(GR4_MODULE_TYPES_${module_id} [==[${module_types}]==])
"
      "set(GR4_MODULE_DEVELOP_TYPES_${module_id} [==[${develop_types}]==])
"
      "set(GR4_MODULE_SYMBOL_TYPES_${module_id} [==[complete;custom]==])
")
  endforeach()
  file(WRITE "${GR4_PACKAGING_DIR}/gr4_bundle_types.cmake" "${types_content}")

  # The icon is generated from the core module's logo, the only image in the
  # workspace, so it is only available once that module's sources are present.
  set(icon_source "${GR4_BUNDLE_ICON_SOURCE}")
  if(icon_source STREQUAL "")
    foreach(name IN LISTS modules)
      _gr4_get_module_property(module_source_dir "${name}" RESOLVED_SOURCE_DIR)
      if(EXISTS "${module_source_dir}/docs/logo.png")
        set(icon_source "${module_source_dir}/docs/logo.png")
        break()
      endif()
    endforeach()
  endif()

  set(icon_command "")
  if(icon_source STREQUAL "")
    message(STATUS "No logo found for the installer icon; the bundle will use Inno Setup's own")
  else()
    find_program(GR4_POWERSHELL_EXECUTABLE NAMES powershell.exe DOC "Windows PowerShell" REQUIRED)
    set(icon_command
        COMMAND
        "${GR4_POWERSHELL_EXECUTABLE}" -NoProfile -ExecutionPolicy Bypass -File
        "${GR4_PACKAGING_SCRIPT_DIR}/MakeGr4Icon.ps1" -Logo "${icon_source}" -OutFile
        "${GR4_PACKAGING_DIR}/gr4.ico")
  endif()

  # module names cannot contain ',' so it is a safe separator on any shell
  list(JOIN modules "," modules_argument)
  add_custom_target(
    gr4-installer
    ${icon_command}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DPACKAGING_DIR:PATH=${GR4_PACKAGING_DIR}"
      "-DGNURADIO4_CMAKE_DIR:PATH=${GR4_INSTALL_LIBDIR_FULL}/cmake/gnuradio4"
      "-DMODULES:STRING=${modules_argument}"
      "-DBUNDLE_VERSION:STRING=${GR4_BUNDLE_VERSION}"
      "-DBUNDLE_APP_ID:STRING=${GR4_BUNDLE_APP_ID}"
      "-DDEBUG_COMPONENTS_DIR:PATH=${GR4_DEBUG_PACKAGING_DIR}"
      "-DOUTPUT_DIR:PATH=${CMAKE_BINARY_DIR}"
      "-DSCRIPT_DIR:PATH=${GR4_PACKAGING_SCRIPT_DIR}"
      "-DTEMPLATE:FILEPATH=${GR4_PACKAGING_SCRIPT_DIR}/gr4_bundle.iss.in"
      -P "${GR4_PACKAGING_SCRIPT_DIR}/GenerateBundleIss.cmake"
    COMMAND "${GR4_ISCC_EXECUTABLE}" /Q "${GR4_PACKAGING_DIR}/gr4_bundle.iss"
    DEPENDS ${package_targets}
    USES_TERMINAL
    VERBATIM
    COMMENT "Composing the unified GNU Radio 4 installer")
endfunction()
