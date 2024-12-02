
#ifndef MYSCENE_HPP
#define MYSCENE_HPP

#include "threepp/threepp.hpp"

using namespace threepp;

struct CanvasParameters {
    float width;
    float height;
};

inline Canvas::Parameters canvasParameter() {//vinduet til programmet
    Canvas::Parameters parameter;
    parameter.title("Inverse Kinematics ");
    parameter.size(800.0f, 800.0f);
    parameter.vsync(true);
    parameter.antialiasing(4);
    parameter.resizable(false);
    return parameter;
}

inline auto createScene() {
    return Scene::create();
}

inline std::shared_ptr<OrthographicCamera> createOrthographicCamera() {

    auto camera = OrthographicCamera::create(
            -20, 20,
            20, -20,
            -100.0f, 100.0f);

    return camera;
}

#endif