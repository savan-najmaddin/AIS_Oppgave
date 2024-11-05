cmake_minimum_required(VERSION 3.19)
project(kinematikk)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)


FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2e2397e0b7f0e639f254ea3046b70e53a71f0387
)
FetchContent_MakeAvailable(threepp)

include(FetchContent)
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.3.2
)
FetchContent_MakeAvailable(Catch2)



add_subdirectory(src)

target_include_directories(kinematikk PUBLIC include)

target_link_libraries(kinematikk PRIVATE threepp::threepp)

enable_testing()
add_subdirectory(tests)
