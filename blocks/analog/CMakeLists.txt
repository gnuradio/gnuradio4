set(GrAnalogBlocks_HDRS
    include/gnuradio-4.0/analog/Agc.hpp
    include/gnuradio-4.0/analog/Agc2.hpp
    include/gnuradio-4.0/analog/AmDemod.hpp
    include/gnuradio-4.0/analog/FmDet.hpp
    include/gnuradio-4.0/analog/FrequencyMod.hpp
    include/gnuradio-4.0/analog/PhaseModulator.hpp
    include/gnuradio-4.0/analog/QuadratureDemod.hpp)

set(GrAnalogBlocks_LIBS gr-analog)

add_library(gr-analog INTERFACE ${GrAnalogBlocks_HDRS})
target_link_libraries(gr-analog INTERFACE gnuradio-core gnuradio-algorithm)
target_include_directories(gr-analog INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                              $<INSTALL_INTERFACE:include/>)

gr_add_block_library(
  GrAnalogBlocks
  MAKE_SHARED_LIBRARY
  MAKE_STATIC_LIBRARY
  HEADERS
  ${GrAnalogBlocks_HDRS}
  LINK_LIBRARIES
  ${GrAnalogBlocks_LIBS})

if(TARGET GrAnalogBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()
