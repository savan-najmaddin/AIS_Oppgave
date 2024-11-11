
#ifndef MINSCENE_HPP
#define MINSCENE_HPP

#include "threepp/threepp.hpp"



using namespace threepp;

inline Canvas::Parameters canvasParameter(){ //vinduet til programmet
    Canvas::Parameters parameter;
    parameter.title("Min tittel");
    parameter.size(800, 800);
    parameter.vsync(true);
    parameter.resizable(false);
    return parameter;

};

inline auto createScene() {
    return Scene::create();
}

inline std::shared_ptr<OrthographicCamera> createOrthographicCamera () {
    float left = -15.0f;
    float right = 15.0f;
    float top = 15.0f;
    float bottom =-15.0f;
    float near = 0.1f;
    float far = 1000.0f;

    auto camera = OrthographicCamera::create(left, right, top, bottom, near, far);
    camera ->position.set(0, 0, 20);
    camera ->lookAt(0, -10, 0);

    return camera;
}

//

#endif //MINSCENE_HPP
