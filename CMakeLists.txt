cmake_minimum_required(VERSION 3.19)
project(kinematikk)

set(CMAKE_CXX_STANDARD 20)

option(KINEMATIKK_TEST "Build tests" ON)

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)

FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2e2397e0b7f0e639f254ea3046b70e53a71f0387
)
FetchContent_MakeAvailable(threepp)
set(Eigen3_DIR "$ENV{EIGEN3_ROOT}/cmake")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if (KINEMATIKK_TEST)
    FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v3.7.1
    )
    FetchContent_MakeAvailable(Catch2)
    enable_testing()
endif ()

add_subdirectory(external)
add_subdirectory(src)
add_subdirectory(tests)

add_executable(kinematikk main.cpp)
target_link_libraries(kinematikk PUBLIC kinematikkLib)
