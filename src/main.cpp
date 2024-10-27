#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
//#include "objekt.hpp"
#include "minScene.hpp"



using namespace threepp;


int main()
{
    auto parameter = canvasParameter();
    Canvas canvas(parameter);

    std::shared_ptr<Scene> scene = createScene();

    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamere();

    canvas.animate([&]() { //skriv kode for vinduinnhold


    });


    return 0;
}


