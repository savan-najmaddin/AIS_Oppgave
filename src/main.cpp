#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include "objekt.hpp"
#include "Logikk.hpp"
#include "Eigen/Core"


using namespace threepp;

//brukes for testing av catch2

int main() {




    auto parameter = canvasParameter();
    Canvas canvas(parameter);

    Clock clock;

    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamera();

    interactivPlane plane(10, 3);
    scene->add(plane.getPlane());
    plane.getPlane()->position.x = 0;
    plane.getPlane()->position.y = 0;

    controller::MyMouseListener ml{clock.elapsedTime};
    canvas.addMouseListener(ml);

  /* kinematicChain armVec(3);

    armVec.joints[0] = Joint(0.0f);
    armVec.joints[1] = Joint(0.0f);
    armVec.joints[2] = Joint(0.0f);
    trenger settet først
    */










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


