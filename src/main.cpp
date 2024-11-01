#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
#include "objekt.hpp"
#include <iostream>
#include "minScene.hpp"
#include "controller.hpp"

using namespace threepp;

//brukes for testiing av catch2
int add(const int x, const int y) {
        return x + y;
    };

int main()
{
int add (5+4); //test funkjson


    auto parameter = canvasParameter();
    Canvas canvas(parameter);

    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamere();

    armSegment arm_1(1, 1, 1);
    armSegment arm_2(1, 1, 1);



    scene->add(arm_1.getSegment());
    scene->add(arm_2.getSegment());


    canvas.animate([&]() {
        renderer.clear();
        renderer.render(*scene, *camera);
        camera -> updateProjectionMatrix();

        arm_2.getSegment()->position.x += 0.005;


        //skriv kode for vinduinnhold

        //legg til kode for render


    });


    return 0;
}


