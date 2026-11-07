# ExternalProject children have independent caches. Define the custom build
# type before project() initializes each language's flags, including in compiler
# checks. Cache values supplied by users still take precedence over these defaults.
foreach(language C CXX)
  if(CMAKE_${language}_COMPILER_LOADED
     AND NOT DEFINED CMAKE_${language}_FLAGS_RELWITHASSERT_INIT)
    if(MSVC)
      set(CMAKE_${language}_FLAGS_RELWITHASSERT_INIT "/O2 /GL /DASSERT_ENABLED")
    else()
      set(CMAKE_${language}_FLAGS_RELWITHASSERT_INIT
          "-O2 -DASSERT_ENABLED -ffunction-sections -fdata-sections")
    endif()
  endif()
endforeach()
