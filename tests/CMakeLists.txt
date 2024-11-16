add_executable(test_main test_main.cpp)
add_test(NAME test_main COMMAND test_main)

target_link_libraries(
        test_main
        PRIVATE
        Catch2::Catch2
        threepp::threepp

)

target_link_directories(test_main PRIVATE ${eigen_SOURCE_DIR})




#trenger å linke til main.cpp
