# Script mode: generates the unified installer script from the module
# installers collected by CollectComponentInstaller.cmake.
#
#   cmake -DPACKAGING_DIR=... -DGNURADIO4_CMAKE_DIR=<prefix>/lib/cmake/gnuradio4
#         -DMODULES=a,b,c -DBUNDLE_VERSION=... -DBUNDLE_APP_ID=...
#         -DDEBUG_COMPONENTS_DIR=... -DOUTPUT_DIR=... -DSCRIPT_DIR=...
#         -DTEMPLATE=.../gr4_bundle.iss.in -P GenerateBundleIss.cmake
#
# This runs at build time because the installers, their versions and the
# installed GR 4 installer templates do not exist until the modules are built.
# MODULES is the configure-time selection in install order; installers left in
# PACKAGING_DIR by an earlier, different selection are ignored.

foreach(variable PACKAGING_DIR GNURADIO4_CMAKE_DIR MODULES BUNDLE_VERSION BUNDLE_APP_ID OUTPUT_DIR SCRIPT_DIR
                 TEMPLATE)
  if(NOT DEFINED ${variable} OR "${${variable}}" STREQUAL "")
    message(FATAL_ERROR "GenerateBundleIss.cmake requires -D${variable}=...")
  endif()
endforeach()
string(REPLACE "," ";" modules "${MODULES}")

# The install types, and each module's membership of them, are decided at
# configure time from the module registry, which build time cannot see.
include("${PACKAGING_DIR}/gr4_bundle_types.cmake")

# Reads the value instead of find_package(): the config defines imported
# targets, which script mode cannot do.
set(gnuradio4_config "${GNURADIO4_CMAKE_DIR}/gnuradio4Config.cmake")
if(NOT EXISTS "${gnuradio4_config}")
  message(FATAL_ERROR "Cannot find the installed GNU Radio 4 core: ${gnuradio4_config}")
endif()
file(STRINGS "${gnuradio4_config}" api_version_line REGEX "^set\\(GR_PLUGIN_API_VERSION ")
string(REGEX MATCH "\"([^\"]*)\"" api_version_match "${api_version_line}")
set(GR_PLUGIN_API_VERSION "${CMAKE_MATCH_1}")
if(GR_PLUGIN_API_VERSION STREQUAL "")
  message(FATAL_ERROR "${gnuradio4_config} does not set GR_PLUGIN_API_VERSION")
endif()

# The bundle creates no uninstall entry of its own, but its AppId still keys
# the state Inno Setup keeps for "this is already installed", so it follows the
# same ABI rule as the module installers: one product per plugin ABI.
set(BUNDLE_APP_ID "${BUNDLE_APP_ID}_v${GR_PLUGIN_API_VERSION}")

# The shared install flow, configured without packages of its own: the bundle
# only records the install root, each module installer registers itself.
set(template_dir "${GNURADIO4_CMAKE_DIR}/packaging/innosetup")
set(GR_INSTALLER_COMPONENT_PACKAGES "")
set(GR_INSTALLER_PLUGIN_PACKAGES "")
set(GR_INSTALLER_PRESELECTED_COMPONENTS "")
# The bundle registers no packages of its own, so it records no configuration
# either; the constant still has to be defined for the template.
set(GR_INSTALLER_CONFIGURATION "")
file(TO_NATIVE_PATH "${template_dir}/gr_manifest.ps1" GR_MANIFEST_TOOL)
configure_file("${template_dir}/gr_install_flow.iss.in" "${PACKAGING_DIR}/gr_install_flow.iss" @ONLY)
configure_file("${template_dir}/gr_installer_files.iss.in" "${PACKAGING_DIR}/gr_installer_files.iss" @ONLY)

# The environment scripts the Start Menu shortcuts open. They resolve
# everything from their own install location, so only the plugin API version
# and the version banner are baked in.
set(GR4_BUNDLE_VERSION "${BUNDLE_VERSION}")
foreach(launcher gr4-env.cmd gr4-env.ps1 gr4-shell.cmd gr4-dev-shell.cmd)
  # cmd.exe cannot find a label in a file with LF line endings, and these are
  # Windows-only files, so CRLF is forced rather than left to the input
  configure_file("${SCRIPT_DIR}/${launcher}.in" "${PACKAGING_DIR}/${launcher}" @ONLY NEWLINE_STYLE CRLF)
endforeach()

set(types_section "")
foreach(type_id IN LISTS GR4_BUNDLE_TYPE_IDS)
  set(type_flags "")
  if(type_id STREQUAL "custom")
    set(type_flags "; Flags: iscustom")
  endif()
  string(APPEND types_section
    "Name: \"${type_id}\"; Description: \"${GR4_BUNDLE_TYPE_DESCRIPTION_${type_id}}\"${type_flags}\n")
endforeach()

set(components_section "")
set(files_section "")
set(entries_table "")
set(typical_components "")
set(entry_count 0)
foreach(name IN LISTS modules)
  set(fragment "${PACKAGING_DIR}/components/${name}.cmake")
  if(NOT EXISTS "${fragment}")
    message(STATUS "${name}: no installer was collected, leaving it out of the bundle")
    continue()
  endif()
  unset(GR4_COMPONENT_PACKAGE_NAME)
  unset(GR4_COMPONENT_VERSION)
  unset(GR4_COMPONENT_SUMMARY)
  unset(GR4_COMPONENT_INSTALLER)
  unset(GR4_COMPONENT_CPACK_COMPONENTS)
  unset(GR4_COMPONENT_MANIFEST_PACKAGES)
  include("${fragment}")

  # "gr-core=4.0.0|gr-foo=4.0.0" -> "gr-core,gr-foo". The installer script asks
  # the manifest about these names to find out whether a debug build of this
  # module is already on the machine.
  set(manifest_packages "")
  string(REPLACE "|" ";" manifest_entries "${GR4_COMPONENT_MANIFEST_PACKAGES}")
  foreach(manifest_entry IN LISTS manifest_entries)
    string(REGEX REPLACE "=.*$" "" manifest_entry "${manifest_entry}")
    if(NOT manifest_entry STREQUAL "")
      list(APPEND manifest_packages "${manifest_entry}")
    endif()
  endforeach()
  list(JOIN manifest_packages "," manifest_packages)

  string(MAKE_C_IDENTIFIER "${name}" id)
  set(description "${GR4_COMPONENT_SUMMARY} ${GR4_COMPONENT_VERSION}")
  string(REPLACE "\"" "\"\"" section_description "${description}")
  string(REPLACE "'" "''" pascal_description "${description}")

  # DEVELOP and SYMBOLS get their own checkbox; everything else the module
  # ships is installed with it. A module without CPack components takes no
  # /COMPONENTS at all.
  set(base_components ${GR4_COMPONENT_CPACK_COMPONENTS})
  set(has_develop False)
  set(has_symbols False)
  if("DEVELOP" IN_LIST base_components)
    set(has_develop True)
    list(REMOVE_ITEM base_components DEVELOP)
  endif()
  if("SYMBOLS" IN_LIST base_components)
    set(has_symbols True)
    list(REMOVE_ITEM base_components SYMBOLS)
  endif()
  list(JOIN base_components "," base_components)

  # A debug build of the same module, collected from a second superbuild tree.
  # It carries the same AppId, so Windows keeps one product per module and one
  # uninstall entry covering both; the debug postfix keeps the files apart.
  set(debug_installer "")
  if(NOT "${DEBUG_COMPONENTS_DIR}" STREQUAL "" AND EXISTS "${DEBUG_COMPONENTS_DIR}/${name}.exe")
    set(debug_installer "${name}-debug.exe")
    file(COPY_FILE "${DEBUG_COMPONENTS_DIR}/${name}.exe" "${PACKAGING_DIR}/components/${debug_installer}"
         ONLY_IF_DIFFERENT)
  endif()

  list(JOIN GR4_MODULE_TYPES_${id} " " module_types)
  list(JOIN GR4_MODULE_DEVELOP_TYPES_${id} " " develop_types)
  list(JOIN GR4_MODULE_SYMBOL_TYPES_${id} " " symbol_types)

  string(APPEND components_section
    "Name: \"${id}\"; Description: \"${section_description}\"; Types: ${module_types}\n")
  if("typical" IN_LIST GR4_MODULE_TYPES_${id})
    list(APPEND typical_components "${id}")
  endif()
  if(has_develop)
    string(APPEND components_section
      "Name: \"${id}\\develop\"; Description: \"Development files\"; Types: ${develop_types}\n")
    if("typical" IN_LIST GR4_MODULE_DEVELOP_TYPES_${id})
      list(APPEND typical_components "${id}\\develop")
    endif()
  endif()
  if(has_symbols)
    string(APPEND components_section
      "Name: \"${id}\\symbols\"; Description: \"Debug symbols\"; Types: ${symbol_types}\n")
  endif()
  if(NOT debug_installer STREQUAL "")
    # deliberately in no install type: a debug build is always an explicit choice
    string(APPEND components_section
      "Name: \"${id}\\debug\"; Description: \"Debug binaries\"; Types: custom\n")
  endif()

  string(APPEND files_section
    "Source: \"components\\${GR4_COMPONENT_INSTALLER}\"; DestDir: \"{tmp}\"; Flags: ignoreversion; Components: ${id}\n")
  if(NOT debug_installer STREQUAL "")
    string(APPEND files_section
      "Source: \"components\\${debug_installer}\"; DestDir: \"{tmp}\"; Flags: ignoreversion; Components: ${id}\\debug\n")
  endif()

  string(APPEND entries_table
    "  Gr4Entries[${entry_count}].Name := '${name}';\n"
    "  Gr4Entries[${entry_count}].Id := '${id}';\n"
    "  Gr4Entries[${entry_count}].Installer := '${GR4_COMPONENT_INSTALLER}';\n"
    "  Gr4Entries[${entry_count}].DebugInstaller := '${debug_installer}';\n"
    "  Gr4Entries[${entry_count}].Description := '${pascal_description}';\n"
    "  Gr4Entries[${entry_count}].Components := '${base_components}';\n"
    "  Gr4Entries[${entry_count}].Packages := '${manifest_packages}';\n"
    "  Gr4Entries[${entry_count}].HasDevelop := ${has_develop};\n"
    "  Gr4Entries[${entry_count}].HasSymbols := ${has_symbols};\n")
  math(EXPR entry_count "${entry_count} + 1")
endforeach()

if(entry_count EQUAL 0)
  message(FATAL_ERROR "No module installers were collected in ${PACKAGING_DIR}/components")
endif()
string(PREPEND entries_table "  SetArrayLength(Gr4Entries, ${entry_count});\n")
list(JOIN typical_components "," GR4_BUNDLE_TYPICAL_COMPONENTS)

set(GR4_BUNDLE_TYPES "${types_section}")
set(GR4_BUNDLE_COMPONENTS "${components_section}")
set(GR4_BUNDLE_FILES "${files_section}")
set(GR4_BUNDLE_ENTRIES "${entries_table}")

# The icon is optional: without a logo to generate it from, Inno Setup uses its
# own and the shortcuts inherit the launcher's.
set(GR4_BUNDLE_SETUP_ICON "")
set(GR4_BUNDLE_ICON_FILES "")
set(GR4_BUNDLE_SHORTCUT_ICON "")
if(EXISTS "${PACKAGING_DIR}/gr4.ico")
  set(GR4_BUNDLE_SETUP_ICON "SetupIconFile=gr4.ico")
  set(GR4_BUNDLE_ICON_FILES "Source: \"gr4.ico\"; DestDir: \"{app}\"; Flags: ignoreversion\n")
  set(GR4_BUNDLE_SHORTCUT_ICON "; IconFilename: \"{app}\\gr4.ico\"")
endif()

file(TO_NATIVE_PATH "${PACKAGING_DIR}/gr_installer_files.iss" GR4_INSTALLER_FILES_ISS)
file(TO_NATIVE_PATH "${PACKAGING_DIR}/gr_install_flow.iss" GR4_INSTALL_FLOW_ISS)
file(TO_NATIVE_PATH "${OUTPUT_DIR}" GR4_BUNDLE_OUTPUT_DIR)
set(GR4_BUNDLE_OUTPUT_BASENAME "gnuradio4-${BUNDLE_VERSION}-win64")
configure_file("${TEMPLATE}" "${PACKAGING_DIR}/gr4_bundle.iss" @ONLY)

message(STATUS "Generated ${PACKAGING_DIR}/gr4_bundle.iss with ${entry_count} module installer(s)")
