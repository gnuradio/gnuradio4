# Digital block unit tests

# Test helper library for digital blocks
add_library(gr-digital-test-helpers INTERFACE)
target_include_directories(gr-digital-test-helpers
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/helpers
)
target_link_libraries(gr-digital-test-helpers
    INTERFACE
    gr-digital
    gr-testing
    ut
)

# Core tests (foundation)
add_executable(qa_digital_core
    core/qa_Lfsr.cpp
    # core/qa_Crc.cpp           # TODO: Implement
    # core/qa_Scrambler.cpp     # TODO: Implement  
    # core/qa_Constellation.cpp # TODO: Implement
)
target_link_libraries(qa_digital_core
    PRIVATE
    gr-digital-test-helpers
)
add_test(NAME qa_digital_core COMMAND qa_digital_core)

# TODO: Enable these tests as we implement the corresponding blocks

# # Mapping tests
# add_executable(qa_digital_mapping
#     mapping/qa_ChunksToSymbols.cpp
#     mapping/qa_ConstellationEncoder.cpp
#     mapping/qa_ConstellationSoftDecoder.cpp
#     mapping/qa_DiffCoding.cpp
#     mapping/qa_BinarySlicer.cpp
#     mapping/qa_Map.cpp
# )
# target_link_libraries(qa_digital_mapping
#     PRIVATE
#     gr-digital-test-helpers
# )
# add_test(NAME qa_digital_mapping COMMAND qa_digital_mapping)

# # Timing tests
# add_executable(qa_digital_timing
#     timing/qa_ClockRecoveryMM.cpp
#     timing/qa_SymbolSync.cpp
#     timing/qa_CostasLoop.cpp
#     timing/qa_PfbClockSync.cpp
# )
# target_link_libraries(qa_digital_timing
#     PRIVATE
#     gr-digital-test-helpers
# )
# add_test(NAME qa_digital_timing COMMAND qa_digital_timing)

# # Measurement tests
# add_executable(qa_digital_measure
#     measure/qa_MpskSnrEst.cpp
#     measure/qa_MeasEvm.cpp
#     measure/qa_ProbeDensity.cpp
# )
# target_link_libraries(qa_digital_measure
#     PRIVATE
#     gr-digital-test-helpers
# )
# add_test(NAME qa_digital_measure COMMAND qa_digital_measure)

# # Equalizer tests
# add_executable(qa_digital_equalizer
#     equalizer/qa_AdaptiveAlgorithm.cpp
#     equalizer/qa_LinearEqualizer.cpp
#     equalizer/qa_DFE.cpp
# )
# target_link_libraries(qa_digital_equalizer
#     PRIVATE
#     gr-digital-test-helpers
# )
# add_test(NAME qa_digital_equalizer COMMAND qa_digital_equalizer)

# # OFDM tests
# add_executable(qa_digital_ofdm
#     ofdm/qa_CyclicPrefixer.cpp
#     ofdm/qa_Serializer.cpp
#     ofdm/qa_CarrierAllocator.cpp
#     ofdm/qa_Equalizers.cpp
#     ofdm/qa_ChanEst.cpp
#     ofdm/qa_SyncSC.cpp
#     ofdm/qa_OfdmE2E.cpp
# )
# target_link_libraries(qa_digital_ofdm
#     PRIVATE
#     gr-digital-test-helpers
# )
# add_test(NAME qa_digital_ofdm COMMAND qa_digital_ofdm)

# # Packet tests
# add_executable(qa_digital_packet
#     packet/qa_HeaderFormats.cpp
#     packet/qa_PacketHeaderGenParse.cpp
#     packet/qa_PacketSink.cpp
#     packet/qa_HeaderPayloadDemux.cpp
#     packet/qa_FramerSink.cpp
# )
# target_link_libraries(qa_digital_packet
#     PRIVATE
#     gr-digital-test-helpers
# )
# add_test(NAME qa_digital_packet COMMAND qa_digital_packet)

# # Misc tests
# add_executable(qa_digital_misc
#     misc/qa_CorrelateAccessCode.cpp
#     misc/qa_PnCorrelator.cpp
#     misc/qa_BurstShaper.cpp
#     misc/qa_ProtocolFormatterParser.cpp
#     misc/qa_Hdlc.cpp
# )
# target_link_libraries(qa_digital_misc
#     PRIVATE
#     gr-digital-test-helpers
# )
# add_test(NAME qa_digital_misc COMMAND qa_digital_misc)