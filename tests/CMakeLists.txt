add_executable(tests tests.cpp)
add_test(NAME tests COMMAND tests)

target_link_libraries(tests PRIVATE Catch2::Catch2)


FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.7.1
)
FetchContent_MakeAvailable(Catch2)



enable_testing()

#trenger å linke til main.cpp
