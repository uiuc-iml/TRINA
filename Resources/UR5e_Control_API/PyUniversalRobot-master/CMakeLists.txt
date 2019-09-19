# ref: http://www.benjack.io/2018/02/02/python-cpp-revisited.html

cmake_minimum_required(VERSION 3.0)

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

project(PyUniversalRobot_kinematcs)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Set source directory
set(SOURCE_DIR "src/PyUniversalRobot/kinematics")
# Tell CMake that headers are also in SOURCE_DIR
include_directories(${SOURCE_DIR} ${EIGEN_INCLUDE_DIR})
set(SOURCES "${SOURCE_DIR}/kinematics.cpp" "${SOURCE_DIR}/kinematics.hpp" "${SOURCE_DIR}/accelerator.cpp" "${SOURCE_DIR}/accelerator.hpp")

# Generate Python module
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(kinematics ${SOURCES} "${SOURCE_DIR}/bindings.cpp")
target_link_libraries(kinematics PRIVATE Eigen3::Eigen)

# download data
include(ExternalProject)
ExternalProject_Add(
    data-ur3
    PREFIX data-ur3
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/PyUniversalRobot/data/UR3"
    URL http://people.duke.edu/~mtd13/PyUniversalRobot/data/UR3.zip
    URL_HASH SHA512=ae4fdf255e0671ec8b0a307ce809397898ee86a64721560b794f2a24b52b2ab7c9516a44644f402d183492b7d5a449b92fdb7fc94ac43257b3fc7940b44d8f3f
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
