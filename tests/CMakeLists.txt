add_executable(test_main testLogic.cpp)
target_link_libraries(
        test_main
        PRIVATE
        Catch2::Catch2WithMain
        threepp::threepp
        kinematikkLib
        Eigen3::Eigen
)

include(CTest)
include(Catch)
catch_discover_tests(test_main)