cmake_minimum_required(VERSION 3.19)
project(kinematikk)

set(CMAKE_CXX_STANDARD 20)

option(KINEMATIKK_TEST "Build tests" ON)

include(FetchContent)

add_subdirectory(src)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2e2397e0b7f0e639f254ea3046b70e53a71f0387
)
FetchContent_MakeAvailable(threepp)

add_executable(kinematikk src/main.cpp)

if(KINEMATIKK_TEST)
    FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v3.7.1
    )
    FetchContent_MakeAvailable(Catch2)

    enable_testing()

    add_subdirectory(tests)
endif()

set(eigen_BUILD_EXAMPLES OFF)
set(eigen_BUILD_Tests OFF)
FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif ()


target_include_directories(kinematikk PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(Eigen3 REQUIRED)

target_link_libraries(kinematikk PRIVATE kinematikkLib threepp::threepp Eigen3::Eigen)