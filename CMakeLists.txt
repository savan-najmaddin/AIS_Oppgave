cmake_minimum_required(VERSION 3.19)
project(kinematikk VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

option(KINEMATIKK_TEST "Build tests" ON)

include(FetchContent)

set(THREEPP_BUILD_TESTS)
set(THREEPP_BUILD_EXAMPLES)

FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2e2397e0b7f0e639f254ea3046b70e53a71f0387
)
FetchContent_MakeAvailable(threepp)

find_package(Eigen3 REQUIRED)

if (KINEMATIKK_TEST)
    FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v3.7.1
    )
    FetchContent_MakeAvailable(Catch2)
    enable_testing()
endif ()

add_subdirectory(src)

add_executable(kinematikk src/main.cpp)
target_link_libraries(kinematikk PUBLIC kinematikkLib) #burde dette være PRIVATE?

if (KINEMATIKK_TEST)
    add_subdirectory(tests)
endif ()