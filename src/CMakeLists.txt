add_library(kinematikkLib controller.cpp objekt.cpp Logikk.cpp)

target_include_directories(kinematikkLib PUBLIC ${CMAKE_SOURCE_DIR}/include)

find_package(Eigen3 REQUIRED)
target_link_libraries(kinematikkLib PRIVATE threepp::threepp Eigen3::Eigen)