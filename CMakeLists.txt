cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME TorqueTest)
set(PROJECT_DESCRIPTION "TorqueTest")
set(PROJECT_URL "")

add_compile_definitions(
  PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

configure_file(etc/TorqueTest.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/TorqueTest.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/TorqueTest.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

