#[[
Unified Windows installer for the superbuild.

Every CMake module already builds its own Inno Setup installer through CPack
(see gr-core's GrInnoSetup.cmake). This file does not repackage their files:
it builds each module's installer, embeds all of them in one bundle and lets
the bundle run the selected ones silently, one after another.

    <name>-package   builds one module's installer and copies it, with a small
                     description of it, into ${GR4_PACKAGING_DIR}/components
    gr4-installer    builds every <name>-package, generates gr4_bundle.iss and
                     compiles it with ISCC

The installer's [Types] come from the module GROUPS. Selecting components does
not enforce module dependencies: installing blocks without core is allowed.
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

set(GR4_PACKAGING_ENABLED FALSE)
if(GR_ENABLE_PACKAGING)
  if(WIN32)
    set(GR4_PACKAGING_ENABLED TRUE)
    set(program_files_x86 "ProgramFiles(x86)")
    find_program(
      GR4_ISCC_EXECUTABLE ISCC.exe
      PATHS "$ENV{${program_files_x86}}/Inno Setup 6" "$ENV{ProgramFiles}/Inno Setup 6"
      DOC "Inno Setup command-line compiler"
      REQUIRED)
  else()
    message(STATUS "GR_ENABLE_PACKAGING: the unified installer is only built on Windows")
  endif()
endif()

set_property(GLOBAL PROPERTY GR4_PACKAGE_TARGETS "")

# Adds <name>-package for a CMake module. Call right after its ExternalProject_Add.
function(gr4_package_project name binary_dir)
  if(NOT GR4_PACKAGING_ENABLED)
    return()
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
      -P "${GR4_PACKAGING_SCRIPT_DIR}/CollectComponentInstaller.cmake"
    DEPENDEES install
    ALWAYS TRUE
    EXCLUDE_FROM_MAIN TRUE
    USES_TERMINAL TRUE)
  ExternalProject_Add_StepTargets("${name}" package)

  set_property(GLOBAL APPEND PROPERTY GR4_PACKAGE_TARGETS "${name}-package")
endfunction()

# Writes the configure-time [Types] include and adds gr4-installer. Call once,
# after gr4_realize_modules().
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

  set(groups "")
  foreach(name IN LISTS modules)
    _gr4_get_module_property(module_groups "${name}" GROUPS)
    foreach(group IN LISTS module_groups)
      if(NOT group IN_LIST groups AND NOT group STREQUAL "custom")
        list(APPEND groups "${group}")
      endif()
    endforeach()
  endforeach()

  # A type installs its group plus what that group needs, which is what the
  # same name means in GR4_MODULE_GROUPS.
  set(types_section "[Types]\n")
  foreach(name IN LISTS modules)
    set(module_types_${name} "")
  endforeach()
  foreach(group IN LISTS groups)
    set(members "")
    foreach(name IN LISTS modules)
      _gr4_get_module_property(module_groups "${name}" GROUPS)
      if(group IN_LIST module_groups)
        list(APPEND members "${name}")
      endif()
    endforeach()

    set(members_changed TRUE)
    while(members_changed)
      set(members_changed FALSE)
      foreach(name IN LISTS members)
        _gr4_get_module_property(module_dependencies "${name}" DEPENDS)
        foreach(dependency IN LISTS module_dependencies)
          if(dependency IN_LIST modules AND NOT dependency IN_LIST members)
            list(APPEND members "${dependency}")
            set(members_changed TRUE)
          endif()
        endforeach()
      endforeach()
    endwhile()

    string(MAKE_C_IDENTIFIER "${group}" type_id)
    string(SUBSTRING "${group}" 0 1 first_letter)
    string(SUBSTRING "${group}" 1 -1 other_letters)
    string(TOUPPER "${first_letter}" first_letter)
    string(APPEND types_section
      "Name: \"${type_id}\"; Description: \"${first_letter}${other_letters}\"\n")
    foreach(name IN LISTS members)
      list(APPEND module_types_${name} "${type_id}")
    endforeach()
  endforeach()
  string(APPEND types_section
    "Name: \"custom\"; Description: \"Custom installation\"; Flags: iscustom\n\n")

  # Components are only known once the installers exist, so the build-time
  # generator refers to these instead of the registry.
  foreach(name IN LISTS modules)
    string(MAKE_C_IDENTIFIER "${name}" module_id)
    list(APPEND module_types_${name} custom)
    list(JOIN module_types_${name} " " module_types)
    string(APPEND types_section "#define Gr4Types_${module_id} \"${module_types}\"\n")
  endforeach()

  file(CONFIGURE
    OUTPUT "${GR4_PACKAGING_DIR}/gr4_bundle_types.iss"
    CONTENT "; Generated by gr-super cmake/PackageGr4.cmake, do not edit.\n${types_section}"
    @ONLY)

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
      "-DBUNDLE_APP_ID:STRING=${GR4_BUNDLE_APP_ID}"
      "-DOUTPUT_DIR:PATH=${CMAKE_BINARY_DIR}"
      "-DTEMPLATE:FILEPATH=${GR4_PACKAGING_SCRIPT_DIR}/gr4_bundle.iss.in"
      -P "${GR4_PACKAGING_SCRIPT_DIR}/GenerateBundleIss.cmake"
    COMMAND "${GR4_ISCC_EXECUTABLE}" /Q "${GR4_PACKAGING_DIR}/gr4_bundle.iss"
    DEPENDS ${package_targets}
    USES_TERMINAL
    VERBATIM
    COMMENT "Composing the unified GNU Radio 4 installer")

  if(NOT TARGET package)
    add_custom_target(package DEPENDS gr4-installer)
  endif()
endfunction()
