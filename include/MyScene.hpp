/**
 * @brief this class is responsible for the screen layout
 */

#ifndef MYSCENE_HPP
#define MYSCENE_HPP

#include "threepp/threepp.hpp"
#include "ImGui.hpp"


MyUI ui();
struct CanvasParameters {
    float width;
    float height;
};

inline threepp::Canvas::Parameters canvasParameter() {//vinduet til programmet
    threepp::Canvas::Parameters parameter;
    parameter.title("Inverse Kinematics ");
    parameter.size(800.0f, 800.0f);
    parameter.vsync(true);
    parameter.antialiasing(4);
    parameter.resizable(false);
    return parameter;
}

inline auto createScene() {
    return threepp::Scene::create();
}

inline std::shared_ptr<threepp::OrthographicCamera> createOrthographicCamera() {

    auto camera = threepp::OrthographicCamera::create(
            -20, 20,
            20, -20,
            -100.0f, 100.0f);

    return camera;
}

#endif