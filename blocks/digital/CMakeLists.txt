# Digital Signal Processing Blocks for GNU Radio 4.0
#
# This module provides digital signal processing blocks including:
# - Core primitives (LFSR, CRC, Scramblers, Constellations)
# - Symbol mapping and constellation encoding/decoding
# - Timing recovery and PLLs
# - Measurement and probing blocks
# - Adaptive equalizers
# - OFDM processing blocks
# - Packet handling and framing
# - Miscellaneous correlation and protocol blocks

include(CMakePackageConfigHelpers)

# Digital blocks library
add_library(gr-digital INTERFACE)
add_library(gnuradio::gr-digital ALIAS gr-digital)

target_include_directories(gr-digital
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(gr-digital
    INTERFACE
    gnuradio-core
    gr-basic
)

target_compile_features(gr-digital INTERFACE cxx_std_23)

# Add tests if enabled
if(ENABLE_TESTING)
    add_subdirectory(test)
endif()

# Installation (simplified for now)
install(TARGETS gr-digital
    COMPONENT digital
)

install(DIRECTORY include/
    DESTINATION include
    COMPONENT digital
    FILES_MATCHING PATTERN "*.hpp"
)