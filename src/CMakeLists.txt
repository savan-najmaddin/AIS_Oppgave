add_library(kinematikkLib
        Visual.cpp
        Handler.cpp
        Objects.cpp
        Controller.cpp
        Logic.cpp
        ImGui.cpp
)
target_include_directories(kinematikkLib PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(kinematikkLib PUBLIC
        threepp::threepp
        Eigen3::Eigen
        imgui::imgui
)
