add_library(ramulator-dpi OBJECT)

target_sources(
  ramulator-dpi PRIVATE
  ramulator_dpi.h
  ramulator_dpi.cpp
)

target_link_libraries(
  ramulator
  PRIVATE
  ramulator-dpi
)

# Standalone shared library for external (C) consumers
add_library(ramulator_dpi_shared SHARED
  ramulator_dpi.cpp
)

target_link_libraries(ramulator_dpi_shared
  PRIVATE ramulator
)

set_target_properties(ramulator_dpi_shared PROPERTIES
  OUTPUT_NAME ramulator_dpi
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)

# Test executable (C source compiled as C++ since the project only enables CXX)
set_source_files_properties(
  ${CMAKE_SOURCE_DIR}/src/dpi_wrapper/test_dpi.c
  PROPERTIES LANGUAGE CXX
)

add_executable(test_dpi
  ${CMAKE_SOURCE_DIR}/src/dpi_wrapper/test_dpi.c
)

target_include_directories(test_dpi PRIVATE
  ${CMAKE_SOURCE_DIR}/src/dpi_wrapper
)

target_link_libraries(test_dpi
  PRIVATE ramulator_dpi_shared
)

set_target_properties(test_dpi PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
  LINKER_LANGUAGE CXX
)
