cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME TorqueTest)
set(PROJECT_DESCRIPTION "TorqueTest")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)  

# Set the CMake prefix path for libtorch
set(CMAKE_PREFIX_PATH "/home/alice/Downloads/libtorch")

set(CMAKE_INSTALL_RPATH "/home/alice/Downloads/libtorch/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Find Torch package
find_package(Torch REQUIRED)

get_target_property(TARGET_INCLUDE_DIRECTORIES torch INTERFACE_INCLUDE_DIRECTORIES)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/TorqueTest.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/TorqueTest.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/TorqueTest.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

