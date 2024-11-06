#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include "objekt.hpp"
#include <vector>


using namespace threepp;

//brukes for testing av catch2

int main() {




    auto parameter = canvasParameter();
    Canvas canvas(parameter);

    Clock clock;

    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamera();

    controller::MyMouseListener ml{clock.elapsedTime};
    canvas.addMouseListener(ml);

    std::vector<armSegment> armVec;
    armVec.emplace_back(1, 5, 0); //armVec[0]
    armVec.emplace_back(1, 5, 0); //armVec[1]

    armVec[0].getSegment()->position.x=0;
    armVec[1].getSegment()->position.x=armVec[0].getSegment()->position.x+10;


    canvas.addMouseListener( ml);



    for (auto& segment : armVec) { //legger til instanser av armvec o scemem
        scene->add(segment.getSegment());
    }



    canvas.animate([&]() {
        renderer.clear();
        renderer.render(*scene, *camera);
        camera->updateProjectionMatrix();


    });


    return 0;
}


