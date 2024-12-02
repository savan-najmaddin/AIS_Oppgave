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

# https://stackoverflow.com/questions/77210209/how-to-prevent-eigen-targets-to-show-up-in-the-main-app-in-a-cmake-project
#used this code to avoid eigen targets to show up in the main app
FetchContent_Declare(Eigen3 SYSTEM
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c # 3.4.0
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR cmake
)

FetchContent_MakeAvailable(Eigen3)
if (NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${eigen3_SOURCE_DIR})
endif ()

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