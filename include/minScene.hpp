
#ifndef MINSCENE_HPP
#define MINSCENE_HPP

#include "threepp/threepp.hpp"



using namespace threepp;

inline Canvas::Parameters canvasParameter(){ //vinduet til programmet
    Canvas::Parameters parameter;
    parameter.title("Min tittel");
    parameter.size(1280, 720);
    parameter.vsync(true);
    parameter.resizable(true);
    return parameter;

};

inline auto createScene() {
    return Scene::create();
}

inline std::shared_ptr<OrthographicCamera> createOrthographicCamere () {
    float left = -5.0f;
    float right = 5.0f;
    float top = 5.0f;
    float bottom = 0.1f;
    float near = 0.1f;
    float far = 1000.0f;

    auto camera = OrthographicCamera::create(left, right, top, bottom, near, far);
    camera ->position.set(0, 0, 10);
    camera ->lookAt(0, 0, 0);

    return camera;
}


#endif //MINSCENE_HPP
