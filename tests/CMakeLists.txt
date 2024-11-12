add_executable(tests tests.cpp test_main.cpp)
add_test(NAME test_main COMMAND test_main)
target_link_libraries(tests PRIVATE Catch2::Catch2 threepp)




#trenger å linke til main.cpp
