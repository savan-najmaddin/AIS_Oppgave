function(add_test_executable name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE
            threepp
            Catch2::Catch2WithMain
            kinematikkLib
            imgui::imgui
            )

    target_include_directories(${name} PRIVATE "${PROJECT_SOURCE_DIR}/src")
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_test_executable(testHandler)
add_test_executable(testLogic)