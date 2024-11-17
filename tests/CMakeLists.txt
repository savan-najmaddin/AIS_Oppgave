add_executable(test_main testLogic.cpp)
target_link_libraries(
        test_main
        PRIVATE
        Catch2::Catch2WithMain
        threepp::threepp
        eigen::eigen
        kinematikkLib

)
add_test(NAME test_main COMMAND test_main)
