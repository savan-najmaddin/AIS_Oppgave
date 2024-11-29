add_executable(test_main testLogic.cpp testHandler.cpp)

find_package(eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(
        test_main
        PRIVATE
        Catch2::Catch2WithMain
        threepp::threepp
        kinematikkLib

)
add_test(NAME test_main COMMAND test_main)

