#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
//#include "objekt.hpp"
#include "minScene.hpp"

#include <objekt.hpp>


using namespace threepp;


int main()
{
    auto parameter = canvasParameter();
    Canvas canvas(parameter);

    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamere();

    armSegment arm_1(1, 1, 1);
    scene->add(arm_1.getSegment());


    canvas.animate([&]() {
        renderer.clear();
        renderer.render(*scene, *camera);



        //skriv kode for vinduinnhold

        //legg til kode for render


    });


    return 0;
}


