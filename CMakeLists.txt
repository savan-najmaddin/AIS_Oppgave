cmake_minimum_required(VERSION 3.19)
project(kinematikk)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

#threepp konfig
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)


FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2e2397e0b7f0e639f254ea3046b70e53a71f0387
)
FetchContent_MakeAvailable(threepp)



#eigen konfig
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

#ferdig konfig

add_subdirectory(libs)
add_subdirectory(src)

target_include_directories(kinematikk PUBLIC include)
#target_include_directories(kinematikk PUBLIC ${eigen_SOURCE_DIR})
# eller link_libraries Eigen3::Eigen
target_link_libraries(kinematikk PRIVATE threepp::threepp)




add_subdirectory(tests)
