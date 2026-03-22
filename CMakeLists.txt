cmake_minimum_required(VERSION 3.11...4.3)
project(UavUgvCoopLocalization VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic -ffast-math)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -fsanitize=address)
    add_link_options(-fsanitize=address)
endif()

# Quiet Eigen + fast-math warnings on Apple Silicon Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang")
    add_compile_options(-Wno-nan-infinity-disabled)
endif()

# All dependencies fetched automatically (no local install)
include(FetchContent)

# Fetch Eigen automatically (header-only)
include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0  # Stable, C++20-friendly
)
FetchContent_MakeAvailable(Eigen)

# Boost (header-only)
FetchContent_Declare(
    Boost
    URL https://archives.boost.io/release/1.90.0/source/boost_1_90_0.tar.gz
    URL_HASH SHA256=5e93d582aff26868d581a52ae78c7d8edf3f3064742c6e77901a1f18a437eea9
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(Boost)

add_library(Boost::headers INTERFACE IMPORTED)
target_include_directories(Boost::headers INTERFACE ${boost_SOURCE_DIR})

# Main library for shared sim code
add_library(vehicle_lib STATIC
    src/filters/estimator.cpp
    src/system/dynamics.cpp
    src/data/export_utils.cpp
)
target_include_directories(vehicle_lib PUBLIC include)

target_link_libraries(vehicle_lib PUBLIC
    Eigen3::Eigen
    Boost::headers
)
target_compile_features(vehicle_lib PUBLIC cxx_std_20)

# Executable: the simulator
add_executable(uav_ugv_coop_localization src/main.cpp)
target_link_libraries(uav_ugv_coop_localization PRIVATE vehicle_lib)

# For future: easy testing
enable_testing()