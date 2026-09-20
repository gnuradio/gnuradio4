# Script mode: generates the unified installer script from the module
# installers collected by CollectComponentInstaller.cmake.
#
#   cmake -DPACKAGING_DIR=... -DGNURADIO4_CMAKE_DIR=<prefix>/lib/cmake/gnuradio4
#         -DMODULES=a,b,c -DBUNDLE_VERSION=... -DBUNDLE_APP_ID=...
#         -DOUTPUT_DIR=... -DTEMPLATE=.../gr4_bundle.iss.in -P GenerateBundleIss.cmake
#
# This runs at build time because the installers, their versions and the
# installed GR 4 installer templates do not exist until the modules are built.
# MODULES is the configure-time selection in install order; installers left in
# PACKAGING_DIR by an earlier, different selection are ignored.

foreach(variable PACKAGING_DIR GNURADIO4_CMAKE_DIR MODULES BUNDLE_VERSION BUNDLE_APP_ID OUTPUT_DIR TEMPLATE)
  if(NOT DEFINED ${variable} OR "${${variable}}" STREQUAL "")
    message(FATAL_ERROR "GenerateBundleIss.cmake requires -D${variable}=...")
  endif()
endforeach()
string(REPLACE "," ";" modules "${MODULES}")

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

# The shared install flow, configured without packages of its own: the bundle
# only records the install root, each module installer registers itself.
set(template_dir "${GNURADIO4_CMAKE_DIR}/packaging/innosetup")
set(GR_INSTALLER_COMPONENT_PACKAGES "")
set(GR_INSTALLER_PLUGIN_PACKAGES "")
set(GR_INSTALLER_PRESELECTED_COMPONENTS "")
file(TO_NATIVE_PATH "${template_dir}/gr_manifest.ps1" GR_MANIFEST_TOOL)
configure_file("${template_dir}/gr_install_flow.iss.in" "${PACKAGING_DIR}/gr_install_flow.iss" @ONLY)
configure_file("${template_dir}/gr_installer_files.iss.in" "${PACKAGING_DIR}/gr_installer_files.iss" @ONLY)

set(components_section "")
set(files_section "")
set(entries_table "")
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
  include("${fragment}")

  string(MAKE_C_IDENTIFIER "${name}" id)
  set(description "${GR4_COMPONENT_SUMMARY} ${GR4_COMPONENT_VERSION}")
  string(REPLACE "\"" "\"\"" section_description "${description}")
  string(REPLACE "'" "''" pascal_description "${description}")

  # DEVELOP gets its own checkbox; everything else the module ships is installed
  # with it. A module without CPack components takes no /COMPONENTS at all.
  set(base_components ${GR4_COMPONENT_CPACK_COMPONENTS})
  set(has_develop False)
  if("DEVELOP" IN_LIST base_components)
    set(has_develop True)
    list(REMOVE_ITEM base_components DEVELOP)
  endif()
  list(JOIN base_components "," base_components)

  string(APPEND components_section
    "Name: \"${id}\"; Description: \"${section_description}\"; Types: {#Gr4Types_${id}}\n")
  if(has_develop)
    string(APPEND components_section
      "Name: \"${id}\\develop\"; Description: \"Development files\"; Types: {#Gr4Types_${id}}\n")
  endif()
  string(APPEND files_section
    "Source: \"components\\${GR4_COMPONENT_INSTALLER}\"; DestDir: \"{tmp}\"; Flags: ignoreversion; Components: ${id}\n")
  string(APPEND entries_table
    "  Gr4Entries[${entry_count}].Name := '${name}';\n"
    "  Gr4Entries[${entry_count}].Id := '${id}';\n"
    "  Gr4Entries[${entry_count}].Installer := '${GR4_COMPONENT_INSTALLER}';\n"
    "  Gr4Entries[${entry_count}].Description := '${pascal_description}';\n"
    "  Gr4Entries[${entry_count}].Components := '${base_components}';\n"
    "  Gr4Entries[${entry_count}].HasDevelop := ${has_develop};\n")
  math(EXPR entry_count "${entry_count} + 1")
endforeach()

if(entry_count EQUAL 0)
  message(FATAL_ERROR "No module installers were collected in ${PACKAGING_DIR}/components")
endif()
string(PREPEND entries_table "  SetArrayLength(Gr4Entries, ${entry_count});\n")

set(GR4_BUNDLE_COMPONENTS "${components_section}")
set(GR4_BUNDLE_FILES "${files_section}")
set(GR4_BUNDLE_ENTRIES "${entries_table}")
file(TO_NATIVE_PATH "${PACKAGING_DIR}/gr_installer_files.iss" GR4_INSTALLER_FILES_ISS)
file(TO_NATIVE_PATH "${PACKAGING_DIR}/gr_install_flow.iss" GR4_INSTALL_FLOW_ISS)
file(TO_NATIVE_PATH "${PACKAGING_DIR}/gr4_bundle_types.iss" GR4_BUNDLE_TYPES_ISS)
file(TO_NATIVE_PATH "${OUTPUT_DIR}" GR4_BUNDLE_OUTPUT_DIR)
set(GR4_BUNDLE_OUTPUT_BASENAME "gnuradio4-${BUNDLE_VERSION}-win64")
configure_file("${TEMPLATE}" "${PACKAGING_DIR}/gr4_bundle.iss" @ONLY)

message(STATUS "Generated ${PACKAGING_DIR}/gr4_bundle.iss with ${entry_count} module installer(s)")
