add_library(kinematikkLib)
target_sources(kinematikkLib PRIVATE controller.cpp objekt.cpp Logikk.cpp)
target_include_directories(kinematikkLib PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(kinematikkLib PUBLIC threepp::threepp eigen::eigen)
