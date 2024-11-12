#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
#include "Eigen/Core"
#include "Logikk.hpp"
#include "controller.hpp"
#include "jointMesh.hpp"
#include "minScene.hpp"
#include "objekt.hpp"



using namespace threepp;

//int add(int x, int y) {
//    return x+y;
//}

int main() {

   //#add(5,4);
   ///std::cout<<add;

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
    circle.getMesh()->position.z = 3.0f;
    scene->add(circle.getMesh());

    kinematicChain chain(3);

    Eigen::Vector2f targetPosition = {2.0f, 3.0f};
    //if (mouselistner) { ny targetPosition}

    Joint joint1(M_PI/2, 2.0f);
    Joint joint2(0.0, 2.0f);
    Joint joint3(0.0f, 2.0f);



    canvas.animate([&]() {

        //chain.updateInverseKinematics(targetPosition, 0.1f);
        scene->clear();
        camera->updateProjectionMatrix();
        renderer.render(*scene, *camera);



    });


    return 0;
}


