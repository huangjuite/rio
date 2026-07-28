# Python bindings for pyekf_rio

find_package(Python REQUIRED COMPONENTS Interpreter Development)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR
  RESULT_VARIABLE NB_RESULT)
message(STATUS "nanobind cmake dir: ${NB_DIR}")
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

set_property(TARGET pyekf_rio_core PROPERTY POSITION_INDEPENDENT_CODE ON)

nanobind_add_module(_core bindings.cpp)
target_link_libraries(_core PRIVATE pyekf_rio_core)
target_include_directories(_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

if(NOT DEFINED PYTHON_INSTALL_DIR)
  set(PYTHON_INSTALL_DIR ".")
endif()

install(TARGETS _core
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/pyekf_rio"
)
