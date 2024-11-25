add_library(kinematikkLib)
target_sources(kinematikkLib PRIVATE controller.cpp Logikk.cpp ImGui.cpp)
target_include_directories(kinematikkLib PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(kinematikkLib PUBLIC threepp::threepp eigen::eigen imgui::imgui)
