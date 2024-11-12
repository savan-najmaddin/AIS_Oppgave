#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include "objekt.hpp"
#include "Logikk.hpp"
#include "Eigen/Core"


using namespace threepp;

int add(int x, int y) {
    return x+y;
}

int main() {

    add(5,4);
    std::cout<<add;

    auto parameter = canvasParameter();
    Canvas canvas(parameter);

    Clock clock;

    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamera();



    controller::MyMouseListener ml{clock.elapsedTime};
    canvas.addMouseListener(ml);

    myBox box;
    box.boxSegment(6, 5, 0.001f );
    scene->add(box.getMesh());

    circleObject circle;
    scene->add(circle.getMesh());
  /* kinematicChain armVec(3);

    armVec.joints[0] = Joint(0.0f);
    armVec.joints[1] = Joint(0.0f);
    armVec.joints[2] = Joint(0.0f);
    trenger settet først
    */





    canvas.animate([&]() {
        renderer.clear();
        renderer.render(*scene, *camera);
        camera->updateProjectionMatrix();


    });


    return 0;
}


