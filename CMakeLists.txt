cmake_minimum_required(VERSION 3.11...4.3)
project(UavUgvCoopLocalization VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warnings and debug tools
add_compile_options(-Wall -Wextra -Wpedantic -ffast-math)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native)  # Speed for sim loops/EKF
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -fsanitize=address)  # Debug robust control
    add_link_options(-fsanitize=address)
endif()

# Fetch Eigen automatically (header-only)
include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0  # Stable, C++20-friendly
)
FetchContent_MakeAvailable(Eigen)

FetchContent_Declare(
    Boost
    URL https://archives.boost.io/release/1.90.0/source/boost_1_90_0.tar.gz
    URL_HASH SHA256=5e93d582aff26868d581a52ae78c7d8edf3f3064742c6e77901a1f18a437eea9
)
FetchContent_MakeAvailable(Boost)

# Main library for shared sim code
add_library(vehicle_lib STATIC
    src/filters/ekf.cpp
    src/system/dynamics.cpp
    src/data/export_utils.cpp
)

target_include_directories(vehicle_lib PUBLIC
    include
    ${eigen_SOURCE_DIR}
    ${Boost_SOURCE_DIR}
)
target_compile_features(vehicle_lib PUBLIC cxx_std_20)

# Executable: the simulator
add_executable(uav_ugv_coop_localization src/main.cpp)
target_link_libraries(uav_ugv_coop_localization PRIVATE vehicle_lib)

# For future: easy testing
enable_testing()