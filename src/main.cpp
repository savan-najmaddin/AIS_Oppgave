#include "threepp/threepp.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"
#include <iostream>

using namespace threepp;


int main() {

    Canvas::Parameters parameter;
    parameter.title("Min tittel");
    parameter.size(monitor::monitorSize());
    parameter.vsync(true);
    parameter.resizable(true);

    Canvas canvas(parameter);
    canvas.animate([&]() {

    Scene scene;
        scene.background = Color::white;


        OrthographicCamera camera;

    });


}
