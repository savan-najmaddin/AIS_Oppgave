add_library(kinematikkLib)
target_sources(kinematikkLib PRIVATE controller.cpp Logic.cpp ImGui.cpp Visual.cpp Handler.cpp)
target_include_directories(kinematikkLib PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(kinematikkLib PUBLIC threepp::threepp eigen::eigen imgui::imgui)
