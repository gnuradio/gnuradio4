# Script mode: generates the unified installer's distribution script from the
# component packages collected by CollectComponentPackage.cmake.
#
#   cmake -DPACKAGING_DIR=... -DGNURADIO4_CMAKE_DIR=<prefix>/lib/cmake/gnuradio4
#         -DMODULES=a,b,c -DBUNDLE_VERSION=... -DMIN_OS_VERSION=...
#         -DHOST_ARCHITECTURES=... -DTEMPLATE=.../gr4_distribution.xml.in
#         [-DLICENSE_FILE=...] -P GenerateBundleDistribution.cmake
#
# This is the macOS counterpart of GenerateBundleIss.cmake and runs at build
# time for the same reason: the packages and their versions do not exist until
# the modules are built. MODULES is the configure-time selection in install
# order; packages left in PACKAGING_DIR by an earlier, different selection are
# ignored.
#
# Unlike the Windows bundle this one configures none of gr-core's installer
# templates. There is no install root to negotiate on macOS, and each component
# package already carries the scripts that register it, so the bundle is purely
# a container.

foreach(variable PACKAGING_DIR GNURADIO4_CMAKE_DIR MODULES BUNDLE_VERSION MIN_OS_VERSION HOST_ARCHITECTURES TEMPLATE)
  if(NOT DEFINED ${variable} OR "${${variable}}" STREQUAL "")
    message(FATAL_ERROR "GenerateBundleDistribution.cmake requires -D${variable}=...")
  endif()
endforeach()
string(REPLACE "," ";" modules "${MODULES}")

# Not used in the distribution itself: this is the check that the GR 4 core was
# actually installed into the shared prefix before the bundle was composed. The
# value is read the same way GenerateBundleIss.cmake reads it, because script
# mode cannot find_package() a config that defines imported targets.
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

# The counterpart of the doubled quotes and apostrophes GenerateBundleIss.cmake
# needs for the Inno Setup sections, and the same class of bug if forgotten: a
# module summary containing an ampersand would otherwise produce a distribution
# file productbuild refuses to parse.
function(xml_escape output_variable text)
  string(REPLACE "&" "&amp;" text "${text}")
  string(REPLACE "<" "&lt;" text "${text}")
  string(REPLACE ">" "&gt;" text "${text}")
  string(REPLACE "\"" "&quot;" text "${text}")
  set(${output_variable} "${text}" PARENT_SCOPE)
endfunction()

# Module groups, written at configure time by PackageGr4.cmake. macOS Installer
# has no install-type concept, so unlike the Windows [Types] section the groups
# only decide which choices start ticked.
set(groups_file "${PACKAGING_DIR}/gr4_bundle_groups.cmake")
if(EXISTS "${groups_file}")
  include("${groups_file}")
endif()

set(choices_outline "")
set(choices "")
set(pkg_refs "")
set(entry_count 0)

foreach(name IN LISTS modules)
  set(fragment "${PACKAGING_DIR}/components/${name}.cmake")
  if(NOT EXISTS "${fragment}")
    message(STATUS "${name}: no component package was collected, leaving it out of the bundle")
    continue()
  endif()
  unset(GR4_COMPONENT_PACKAGE_NAME)
  unset(GR4_COMPONENT_VERSION)
  unset(GR4_COMPONENT_SUMMARY)
  unset(GR4_COMPONENT_PKGS)
  include("${fragment}")

  string(MAKE_C_IDENTIFIER "${name}" id)

  # a module in the base group is what a default installation means here, which
  # is the same thing the "base" type means on Windows
  string(MAKE_C_IDENTIFIER "${name}" group_key)
  set(module_groups "${GR4_MODULE_GROUPS_${group_key}}")
  if("base" IN_LIST module_groups)
    set(start_selected "true")
  else()
    set(start_selected "false")
  endif()

  set(description "${GR4_COMPONENT_SUMMARY} ${GR4_COMPONENT_VERSION}")
  xml_escape(description "${description}")

  # DEVELOP gets its own nested choice; everything else the module ships is
  # installed with it, exactly as the Windows bundle splits the components.
  set(module_refs "")
  set(develop_refs "")
  set(develop_title "")
  foreach(entry IN LISTS GR4_COMPONENT_PKGS)
    string(REPLACE "|" ";" fields "${entry}")
    list(GET fields 0 component)
    list(GET fields 1 package_file)
    list(GET fields 2 identifier)
    list(GET fields 3 display)

    xml_escape(identifier_escaped "${identifier}")
    xml_escape(package_file_escaped "${package_file}")
    string(APPEND pkg_refs
           "    <pkg-ref id=\"${identifier_escaped}\" version=\"${GR4_COMPONENT_VERSION}\" onConclusion=\"none\">${package_file_escaped}</pkg-ref>\n")

    if(component STREQUAL "DEVELOP")
      string(APPEND develop_refs "        <pkg-ref id=\"${identifier_escaped}\"/>\n")
      xml_escape(develop_title "${display}")
    else()
      string(APPEND module_refs "        <pkg-ref id=\"${identifier_escaped}\"/>\n")
    endif()
  endforeach()

  string(APPEND choices
         "    <choice id=\"${id}\" title=\"${description}\" visible=\"true\" start_selected=\"${start_selected}\">\n"
         "${module_refs}"
         "    </choice>\n")

  # A module with no development files takes a self-closing outline entry; the
  # nested form is only meaningful when there is a child choice to nest.
  if(develop_refs STREQUAL "")
    string(APPEND choices_outline "        <line choice=\"${id}\"/>\n")
  else()
    if(develop_title STREQUAL "")
      set(develop_title "Development files")
    endif()
    string(APPEND choices_outline
           "        <line choice=\"${id}\">\n"
           "            <line choice=\"${id}_develop\"/>\n"
           "        </line>\n")
    string(APPEND choices
           "    <choice id=\"${id}_develop\" title=\"${develop_title}\" visible=\"true\" start_selected=\"${start_selected}\">\n"
           "${develop_refs}"
           "    </choice>\n")
  endif()

  math(EXPR entry_count "${entry_count} + 1")
endforeach()

if(entry_count EQUAL 0)
  message(FATAL_ERROR "No module component packages were collected in ${PACKAGING_DIR}/components")
endif()

# productbuild resolves <license file="..."> against --resources, so the file is
# staged there rather than referenced where it lives.
set(GR4_BUNDLE_LICENSE "")
if(DEFINED LICENSE_FILE AND EXISTS "${LICENSE_FILE}")
  file(MAKE_DIRECTORY "${PACKAGING_DIR}/resources")
  file(COPY_FILE "${LICENSE_FILE}" "${PACKAGING_DIR}/resources/LICENSE" ONLY_IF_DIFFERENT)
  set(GR4_BUNDLE_LICENSE "    <license file=\"LICENSE\"/>\n")
endif()

set(GR4_BUNDLE_TITLE "GNU Radio 4")
set(GR4_MACOS_MIN_VERSION "${MIN_OS_VERSION}")
set(GR4_MACOS_HOST_ARCHITECTURES "${HOST_ARCHITECTURES}")
set(GR4_BUNDLE_CHOICES_OUTLINE "${choices_outline}")
set(GR4_BUNDLE_CHOICES "${choices}")
set(GR4_BUNDLE_PKG_REFS "${pkg_refs}")
configure_file("${TEMPLATE}" "${PACKAGING_DIR}/gr4_distribution.xml" @ONLY NEWLINE_STYLE LF)

message(STATUS "Generated ${PACKAGING_DIR}/gr4_distribution.xml with ${entry_count} module(s)")
