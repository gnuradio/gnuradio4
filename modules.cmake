# Ordered GNU Radio 4 module manifest. Dependencies must be declared before
# consumers so the generated ExternalProject graph remains easy to inspect.

set(gr4_core_dependencies "")
if(GR4_FETCH_DEPS)
  gr4_register_module(
    NAME gnuradio4-deps
    TYPE CMAKE
    SOURCE_DIR gnuradio4-deps
    SOURCE_KEY DEPS
    OPTIONS_KEY DEPS
    REPOSITORY https://github.com/gnuradio/gnuradio4-deps.git
    REF main
    CMAKE_ARGS
      "-DTARGET_GR_CORE:BOOL=ON"
      "-DTARGET_GR_BLOCKS:BOOL=$<TARGET_EXISTS:gnuradio4-blocks>"
      "-DTARGET_GR_CONTROL_PLANE:BOOL=$<TARGET_EXISTS:gnuradio4-control-plane>"
      "-DTARGET_GR_STUDIO:BOOL=$<TARGET_EXISTS:gnuradio4-studio-blocks>"
      "-DTARGET_TESTS:BOOL=${GR4_BUILD_TESTING}"
      # Core and blocks configure Boost.UT even with tests disabled.
      "-DENABLE_BOOST_UT:BOOL=ON")
  set(gr4_core_dependencies DEPENDS gnuradio4-deps)
endif()

gr4_register_module(
  NAME gnuradio4-core
  TYPE CMAKE
  SOURCE_DIR gnuradio4-core
  SOURCE_KEY CORE
  OPTIONS_KEY CORE
  REPOSITORY https://github.com/gnuradio/gnuradio4-core.git
  REF main
  GROUPS base
  ${gr4_core_dependencies})

gr4_register_module(
  NAME gnuradio4-blocks
  TYPE CMAKE
  SOURCE_DIR gnuradio4-blocks
  SOURCE_KEY BLOCKS
  OPTIONS_KEY BLOCKS
  REPOSITORY https://github.com/gnuradio/gnuradio4-blocks.git
  REF main
  GROUPS base
  DEPENDS gnuradio4-core)

gr4_register_module(
  NAME gr4-incubator
  TYPE CMAKE
  SOURCE_DIR gr4-incubator
  SOURCE_KEY INCUBATOR
  OPTIONS_KEY INCUBATOR
  REPOSITORY https://github.com/gnuradio/gr4-incubator.git
  REF main
  GROUPS experimental
  DEPENDS gnuradio4-blocks)

gr4_register_module(
  NAME gnuradio4-control-plane
  TYPE CMAKE
  SOURCE_DIR gnuradio4-control-plane
  SOURCE_KEY CONTROL_PLANE
  OPTIONS_KEY CONTROL_PLANE
  REPOSITORY https://github.com/gnuradio/gnuradio4-control-plane.git
  REF main
  GROUPS full applications
  DEPENDS gnuradio4-blocks)

gr4_register_module(
  NAME gnuradio4-studio-blocks
  TYPE CMAKE
  SOURCE_DIR gnuradio4-studio
  SOURCE_SUBDIR blocks
  SOURCE_KEY STUDIO
  OPTIONS_KEY STUDIO_BLOCKS
  REPOSITORY https://github.com/gnuradio/gnuradio4-studio.git
  REF main
  GROUPS full applications
  DEPENDS gnuradio4-blocks
  CMAKE_ARGS
    "-DCMAKE_CXX_FLAGS_INIT:STRING=-D_GLIBCXX_USE_TBB_PAR_BACKEND=0"
    "-DCMAKE_PROJECT_INCLUDE:FILEPATH=${CMAKE_SOURCE_DIR}/cmake/EnableTesting.cmake")

gr4_register_module(
  NAME gnuradio4-studio
  TYPE NODE
  SOURCE_DIR gnuradio4-studio
  GROUPS full applications
  SOURCE_PROVIDER gnuradio4-studio-blocks
  OPTIONAL_DEPENDS gnuradio4-control-plane)
