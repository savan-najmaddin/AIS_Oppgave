
#ifndef MINSCENE_HPP
#define MINSCENE_HPP

#include "threepp/threepp.hpp"



using namespace threepp;
struct CanvasParameters {
    float width;
    float height;
};

inline Canvas::Parameters canvasParameter(){ //vinduet til programmet
    Canvas::Parameters parameter;
    parameter.title("Min tittel");
    parameter.size(800.0f, 800.0f);
    parameter.vsync(true);
    parameter.antialiasing(4);
    parameter.resizable(false);
    return parameter;

};

inline auto createScene() {
    return Scene::create();
}

inline std::shared_ptr<OrthographicCamera> createOrthographicCamera() {
    float viewSize = 20.0f; //vurderer resizeable kamera, la stå for nå
    float aspectRatio = 1.0f;

    auto camera = OrthographicCamera::create(
        -20, 20,
        20, -20, //aspectrartio er 1
        -100.0f, 100.0f
    );

    return camera;
}

//

#endif //MINSCENE_HPP
