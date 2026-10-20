function(_gr4_set_module_property name property)
  set_property(GLOBAL PROPERTY "GR4_MODULE_${name}_${property}" "${ARGN}")
endfunction()

function(_gr4_get_module_property output name property)
  get_property(value GLOBAL PROPERTY "GR4_MODULE_${name}_${property}")
  set("${output}" "${value}" PARENT_SCOPE)
endfunction()

function(gr4_register_module)
  set(options TESTS)
  set(one_value_args
      NAME
      TYPE
      SOURCE_DIR
      SOURCE_SUBDIR
      SOURCE_PROVIDER
      REPOSITORY
      REF
      SOURCE_KEY
      OPTIONS_KEY)
  set(multi_value_args GROUPS DEPENDS OPTIONAL_DEPENDS CMAKE_ARGS)
  cmake_parse_arguments(
    PARSE_ARGV 0 MODULE "${options}" "${one_value_args}" "${multi_value_args}")

  if(MODULE_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "Module declaration has unknown arguments: ${MODULE_UNPARSED_ARGUMENTS}")
  endif()
  if(MODULE_KEYWORDS_MISSING_VALUES)
    message(FATAL_ERROR
      "Module declaration has keywords without values: ${MODULE_KEYWORDS_MISSING_VALUES}")
  endif()

  if(NOT MODULE_NAME)
    message(FATAL_ERROR "gr4_register_module requires NAME")
  endif()
  if(NOT MODULE_NAME MATCHES "^[A-Za-z0-9][A-Za-z0-9_.+-]*$")
    message(FATAL_ERROR "Invalid module name '${MODULE_NAME}'")
  endif()
  if(NOT MODULE_TYPE)
    set(MODULE_TYPE CMAKE)
  endif()
  if(NOT MODULE_TYPE MATCHES "^(CMAKE|NODE)$")
    message(FATAL_ERROR
      "Module '${MODULE_NAME}' has unsupported TYPE '${MODULE_TYPE}'")
  endif()
  if(NOT MODULE_SOURCE_DIR)
    set(MODULE_SOURCE_DIR "${MODULE_NAME}")
  endif()
  if(MODULE_SOURCE_PROVIDER AND NOT MODULE_TYPE STREQUAL "NODE")
    message(FATAL_ERROR
      "Module '${MODULE_NAME}' uses SOURCE_PROVIDER, which is supported only for TYPE NODE")
  endif()
  if(MODULE_SOURCE_PROVIDER STREQUAL MODULE_NAME)
    message(FATAL_ERROR "Module '${MODULE_NAME}' cannot provide its own source")
  endif()
  if(MODULE_SOURCE_PROVIDER AND MODULE_REPOSITORY)
    message(FATAL_ERROR
      "Module '${MODULE_NAME}' cannot use both SOURCE_PROVIDER and REPOSITORY")
  endif()
  if(MODULE_TYPE STREQUAL "NODE"
      AND (MODULE_SOURCE_SUBDIR OR MODULE_OPTIONS_KEY OR MODULE_CMAKE_ARGS))
    message(FATAL_ERROR
      "Node module '${MODULE_NAME}' cannot use SOURCE_SUBDIR, OPTIONS_KEY, or CMAKE_ARGS")
  endif()
  if(MODULE_NAME IN_LIST MODULE_DEPENDS
      OR MODULE_NAME IN_LIST MODULE_OPTIONAL_DEPENDS)
    message(FATAL_ERROR "Module '${MODULE_NAME}' cannot depend on itself")
  endif()
  if(MODULE_SOURCE_PROVIDER
      AND NOT MODULE_SOURCE_PROVIDER IN_LIST MODULE_DEPENDS)
    list(APPEND MODULE_DEPENDS "${MODULE_SOURCE_PROVIDER}")
  endif()
  list(REMOVE_DUPLICATES MODULE_DEPENDS)
  list(REMOVE_DUPLICATES MODULE_OPTIONAL_DEPENDS)

  get_property(module_names GLOBAL PROPERTY GR4_REGISTERED_MODULES)
  if(MODULE_NAME IN_LIST module_names)
    message(FATAL_ERROR "Duplicate module registration '${MODULE_NAME}'")
  endif()

  string(MAKE_C_IDENTIFIER "${MODULE_NAME}" default_key)
  string(TOUPPER "${default_key}" default_key)
  if(NOT MODULE_SOURCE_KEY)
    set(MODULE_SOURCE_KEY "${default_key}")
  endif()
  if(NOT MODULE_OPTIONS_KEY)
    set(MODULE_OPTIONS_KEY "${default_key}")
  endif()
  string(TOUPPER "${MODULE_SOURCE_KEY}" MODULE_SOURCE_KEY)
  string(TOUPPER "${MODULE_OPTIONS_KEY}" MODULE_OPTIONS_KEY)
  foreach(cache_key MODULE_SOURCE_KEY MODULE_OPTIONS_KEY)
    if(NOT "${${cache_key}}" MATCHES "^[A-Z][A-Z0-9_]*$")
      message(FATAL_ERROR
        "Module '${MODULE_NAME}' has invalid cache key '${${cache_key}}'")
    endif()
  endforeach()

  if(MODULE_REPOSITORY)
    if(NOT MODULE_REF)
      set(MODULE_REF main)
    endif()
    set(repository_variable "GR4_${MODULE_SOURCE_KEY}_REPOSITORY")
    set(ref_variable "GR4_${MODULE_SOURCE_KEY}_REF")
    set(
      "${repository_variable}"
      "${MODULE_REPOSITORY}"
      CACHE STRING "Git repository used when ${MODULE_NAME} is missing")
    set(
      "${ref_variable}"
      "${MODULE_REF}"
      CACHE STRING "Git branch, tag, or commit used when cloning ${MODULE_NAME}")
  else()
    set(repository_variable "")
    set(ref_variable "")
  endif()

  if(MODULE_TYPE STREQUAL "CMAKE")
    set(cmake_args_variable "GR4_${MODULE_OPTIONS_KEY}_CMAKE_ARGS")
    set(
      "${cmake_args_variable}"
      ""
      CACHE STRING "Additional CMake arguments passed only to ${MODULE_NAME}")

    # Moves the build tree; empty keeps it below the superbuild's binary
    # directory. A build tree belongs to one module rather than to a checkout,
    # so modules sharing sources still get a variable each.
    set(binary_dir_variable "GR4_${MODULE_OPTIONS_KEY}_BINARY_DIR")
    set(
      "${binary_dir_variable}"
      ""
      CACHE PATH "Build tree for ${MODULE_NAME} instead of the one below the superbuild's binary directory")
  else()
    set(cmake_args_variable "")
    set(binary_dir_variable "")
  endif()

  # Points the module at a checkout anywhere on disk; empty keeps SOURCE_DIR
  # below GR4_SOURCE_ROOT. Like REPOSITORY, it belongs to whoever owns the
  # checkout, so a module with a SOURCE_PROVIDER uses the provider's variable.
  if(MODULE_SOURCE_PROVIDER)
    set(source_dir_variable "")
  else()
    set(source_dir_variable "GR4_${MODULE_SOURCE_KEY}_SOURCE_DIR")
    set(
      "${source_dir_variable}"
      ""
      CACHE PATH "Existing ${MODULE_NAME} source tree to build instead of the one below GR4_SOURCE_ROOT")
  endif()

  set_property(GLOBAL APPEND PROPERTY GR4_REGISTERED_MODULES "${MODULE_NAME}")
  _gr4_set_module_property("${MODULE_NAME}" TYPE "${MODULE_TYPE}")
  _gr4_set_module_property("${MODULE_NAME}" SOURCE_DIR "${MODULE_SOURCE_DIR}")
  _gr4_set_module_property("${MODULE_NAME}" SOURCE_SUBDIR "${MODULE_SOURCE_SUBDIR}")
  _gr4_set_module_property(
    "${MODULE_NAME}" SOURCE_PROVIDER "${MODULE_SOURCE_PROVIDER}")
  _gr4_set_module_property("${MODULE_NAME}" REPOSITORY_VARIABLE "${repository_variable}")
  _gr4_set_module_property("${MODULE_NAME}" REF_VARIABLE "${ref_variable}")
  _gr4_set_module_property("${MODULE_NAME}" SOURCE_DIR_VARIABLE "${source_dir_variable}")
  _gr4_set_module_property("${MODULE_NAME}" CMAKE_ARGS_VARIABLE "${cmake_args_variable}")
  _gr4_set_module_property("${MODULE_NAME}" BINARY_DIR_VARIABLE "${binary_dir_variable}")
  _gr4_set_module_property("${MODULE_NAME}" CMAKE_ARGS "${MODULE_CMAKE_ARGS}")
  _gr4_set_module_property("${MODULE_NAME}" GROUPS "${MODULE_GROUPS}")
  _gr4_set_module_property("${MODULE_NAME}" DEPENDS "${MODULE_DEPENDS}")
  _gr4_set_module_property(
    "${MODULE_NAME}" OPTIONAL_DEPENDS "${MODULE_OPTIONAL_DEPENDS}")
  _gr4_set_module_property("${MODULE_NAME}" TESTS "${MODULE_TESTS}")
endfunction()

function(gr4_select_modules output)
  get_property(module_names GLOBAL PROPERTY GR4_REGISTERED_MODULES)
  if(NOT module_names)
    message(FATAL_ERROR "No GNU Radio 4 modules were registered")
  endif()

  set(known_groups "")
  foreach(name IN LISTS module_names)
    _gr4_get_module_property(module_groups "${name}" GROUPS)
    list(APPEND known_groups ${module_groups})
  endforeach()
  list(REMOVE_DUPLICATES known_groups)

  foreach(group IN LISTS GR4_MODULE_GROUPS)
    if(NOT group IN_LIST known_groups)
      message(FATAL_ERROR
        "Unknown module group '${group}'. Known groups: ${known_groups}")
    endif()
  endforeach()

  set(selected_modules ${GR4_MODULES})
  foreach(name IN LISTS module_names)
    _gr4_get_module_property(module_groups "${name}" GROUPS)
    foreach(group IN LISTS GR4_MODULE_GROUPS)
      if(group IN_LIST module_groups)
        list(APPEND selected_modules "${name}")
        break()
      endif()
    endforeach()
  endforeach()

  # Compatibility with profiles written before module groups were introduced.
  foreach(legacy_mapping
      "GR4_ENABLE_INCUBATOR|gr4-incubator"
      "GR4_ENABLE_CONTROL_PLANE|gnuradio4-control-plane"
      "GR4_ENABLE_STUDIO|gnuradio4-studio-blocks,gnuradio4-studio")
    string(REPLACE "|" ";" mapping_parts "${legacy_mapping}")
    list(GET mapping_parts 0 legacy_variable)
    list(GET mapping_parts 1 legacy_modules)
    string(REPLACE "," ";" legacy_modules "${legacy_modules}")
    if(DEFINED ${legacy_variable} AND NOT "${${legacy_variable}}" STREQUAL "")
      if(${legacy_variable})
        list(APPEND selected_modules ${legacy_modules})
      endif()
    endif()
  endforeach()
  list(REMOVE_DUPLICATES selected_modules)
  list(REMOVE_DUPLICATES GR4_EXCLUDE_MODULES)

  foreach(name IN LISTS selected_modules GR4_EXCLUDE_MODULES)
    if(NOT name IN_LIST module_names)
      message(FATAL_ERROR
        "Unknown module '${name}'. Registered modules: ${module_names}")
    endif()
  endforeach()

  foreach(name IN LISTS selected_modules)
    _gr4_get_module_property(module_dependencies "${name}" DEPENDS)
    _gr4_get_module_property(optional_dependencies "${name}" OPTIONAL_DEPENDS)
    foreach(dependency IN LISTS module_dependencies optional_dependencies)
      if(NOT dependency IN_LIST module_names)
        message(FATAL_ERROR
          "Module '${name}' references unregistered module '${dependency}'")
      endif()
    endforeach()
  endforeach()

  # Add required dependencies until the selection reaches closure.
  set(selection_changed TRUE)
  while(selection_changed)
    set(selection_changed FALSE)
    foreach(name IN LISTS selected_modules)
      _gr4_get_module_property(module_dependencies "${name}" DEPENDS)
      foreach(dependency IN LISTS module_dependencies)
        if(dependency IN_LIST GR4_EXCLUDE_MODULES)
          message(FATAL_ERROR
            "Module '${name}' requires excluded module '${dependency}'")
        endif()
        if(NOT dependency IN_LIST selected_modules)
          list(APPEND selected_modules "${dependency}")
          set(selection_changed TRUE)
        endif()
      endforeach()
    endforeach()
  endwhile()

  list(REMOVE_ITEM selected_modules ${GR4_EXCLUDE_MODULES})

  # Preserve manifest order so dependencies are constructed before consumers.
  set(ordered_modules "")
  foreach(name IN LISTS module_names)
    if(name IN_LIST selected_modules)
      list(APPEND ordered_modules "${name}")
    endif()
  endforeach()

  foreach(name IN LISTS ordered_modules)
    _gr4_get_module_property(module_dependencies "${name}" DEPENDS)
    _gr4_get_module_property(optional_dependencies "${name}" OPTIONAL_DEPENDS)
    foreach(dependency IN LISTS optional_dependencies)
      if(dependency IN_LIST ordered_modules)
        list(APPEND module_dependencies "${dependency}")
      endif()
    endforeach()
    list(FIND module_names "${name}" module_index)
    foreach(dependency IN LISTS module_dependencies)
      list(FIND module_names "${dependency}" dependency_index)
      if(dependency_index GREATER module_index)
        message(FATAL_ERROR
          "Module '${name}' depends on '${dependency}', which is declared later. "
          "Move dependencies before their consumers in the module manifest.")
      endif()
    endforeach()
  endforeach()

  set("${output}" "${ordered_modules}" PARENT_SCOPE)
endfunction()

function(gr4_realize_modules)
  gr4_select_modules(selected_modules)

  foreach(name IN LISTS selected_modules)
    _gr4_get_module_property(module_type "${name}" TYPE)
    _gr4_get_module_property(module_source_dir "${name}" SOURCE_DIR)
    _gr4_get_module_property(module_source_subdir "${name}" SOURCE_SUBDIR)
    _gr4_get_module_property(module_source_provider "${name}" SOURCE_PROVIDER)
    _gr4_get_module_property(repository_variable "${name}" REPOSITORY_VARIABLE)
    _gr4_get_module_property(ref_variable "${name}" REF_VARIABLE)
    _gr4_get_module_property(cmake_args_variable "${name}" CMAKE_ARGS_VARIABLE)
    _gr4_get_module_property(module_cmake_args "${name}" CMAKE_ARGS)
    _gr4_get_module_property(module_dependencies "${name}" DEPENDS)
    _gr4_get_module_property(optional_dependencies "${name}" OPTIONAL_DEPENDS)
    _gr4_get_module_property(module_tests "${name}" TESTS)
    _gr4_get_module_property(source_dir_variable "${name}" SOURCE_DIR_VARIABLE)
    _gr4_get_module_property(binary_dir_variable "${name}" BINARY_DIR_VARIABLE)

    if(module_source_provider)
      # the provider owns the shared checkout, so moving it moves this module too
      _gr4_get_module_property(
        source_dir_variable "${module_source_provider}" SOURCE_DIR_VARIABLE)
    endif()
    set(source_override "")
    if(source_dir_variable)
      set(source_override "${${source_dir_variable}}")
    endif()

    if(source_override STREQUAL "")
      cmake_path(
        ABSOLUTE_PATH module_source_dir
        BASE_DIRECTORY "${GR4_SOURCE_ROOT}"
        NORMALIZE)
    else()
      cmake_path(
        ABSOLUTE_PATH source_override
        BASE_DIRECTORY "${CMAKE_SOURCE_DIR}"
        NORMALIZE)
      set(module_source_dir "${source_override}")

      # an explicit location is never cloned into, so it must already hold the module
      if(module_type STREQUAL "NODE")
        set(source_marker "${module_source_dir}/package.json")
      elseif(module_source_subdir)
        set(source_marker "${module_source_dir}/${module_source_subdir}/CMakeLists.txt")
      else()
        set(source_marker "${module_source_dir}/CMakeLists.txt")
      endif()
      if(NOT EXISTS "${source_marker}")
        message(FATAL_ERROR
          "${source_dir_variable} is '${module_source_dir}', but ${source_marker} does not exist")
      endif()
      message(STATUS "${name}: using sources in ${module_source_dir}")
    endif()

    foreach(dependency IN LISTS optional_dependencies)
      if(dependency IN_LIST selected_modules)
        list(APPEND module_dependencies "${dependency}")
      endif()
    endforeach()

    set(test_argument "")
    if(module_tests)
      set(test_argument TESTS)
    endif()

    if(module_type STREQUAL "CMAKE")
      set(source_subdir_argument "")
      if(module_source_subdir)
        set(source_subdir_argument SOURCE_SUBDIR "${module_source_subdir}")
      endif()

      set(binary_dir_argument "")
      if(binary_dir_variable AND NOT "${${binary_dir_variable}}" STREQUAL "")
        set(module_binary_dir "${${binary_dir_variable}}")
        cmake_path(
          ABSOLUTE_PATH module_binary_dir
          BASE_DIRECTORY "${CMAKE_SOURCE_DIR}"
          NORMALIZE)
        set(binary_dir_argument BINARY_DIR "${module_binary_dir}")
        message(STATUS "${name}: building in ${module_binary_dir}")
      endif()

      set(repository_arguments "")
      if(repository_variable)
        set(repository_arguments
            GIT_REPOSITORY "${${repository_variable}}"
            GIT_TAG "${${ref_variable}}")
      endif()

      set(component_cmake_args ${module_cmake_args})
      if(cmake_args_variable)
        list(APPEND component_cmake_args ${${cmake_args_variable}})
      endif()

      gr4_add_project(
        "${name}"
        SOURCE_DIR "${module_source_dir}"
        ${source_subdir_argument}
        ${binary_dir_argument}
        ${repository_arguments}
        DEPENDS ${module_dependencies}
        CMAKE_ARGS ${component_cmake_args}
        ${test_argument})
    elseif(module_type STREQUAL "NODE")
      set(repository_arguments "")
      if(repository_variable)
        set(repository_arguments
            GIT_REPOSITORY "${${repository_variable}}"
            GIT_TAG "${${ref_variable}}")
      endif()
      gr4_add_node_project(
        "${name}"
        SOURCE_DIR "${module_source_dir}"
        SOURCE_PROVIDER "${module_source_provider}"
        ${repository_arguments}
        DEPENDS ${module_dependencies}
        ${test_argument})
    endif()
  endforeach()

  set(GR4_SELECTED_MODULES "${selected_modules}" PARENT_SCOPE)
endfunction()
