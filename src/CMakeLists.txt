add_executable(kinematikk main.cpp objekt.cpp controller.cpp)

target_link_libraries(kinematikk PRIVATE threepp::threepp)
