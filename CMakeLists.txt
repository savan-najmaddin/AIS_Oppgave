cmake_minimum_required(VERSION 3.19)
project(kinematikk)

set(CMAKE_CXX_STANDARD 20)

option(KINEMATIKK_TEST "Build tests" ON)

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
set(BUILD_TESTING OFF CACHE BOOL "Disable Eigen's tests" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen documentation build" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Disable Eigen's pkg-config files" FORCE)


FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2e2397e0b7f0e639f254ea3046b70e53a71f0387
)
FetchContent_MakeAvailable(threepp)



FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)
add_library(eigen::eigen INTERFACE IMPORTED)
set_target_properties(eigen::eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}"
)

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