#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
#include "Eigen/Core"
#include "Logikk.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include "objekt.hpp"

#include <Visual.hpp>


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

    kinematicChain chain;

    chain.addJoint(Joint(M_PI , 5.0f));
    chain.addJoint(Joint(M_PI , 5.0f));
    chain.addJoint(Joint(M_PI , 5.0f));

    Eigen::Vector2f targetPosition = {2.0f, 3.0f};
    float learningRate = 0.1f;
    //if (mouselistner) { ny targetPosition}



    std::cout << "Joint Positions:\n";
    for (const auto& joint : chain.joints) {
        std::cout << "Angle: " << joint.angle << ", Length: " << joint.length << std::endl;
    }

    Eigen::Vector2f lastPosition = chain.findEffectorPosition();
    std::cout<< "Last position: " << lastPosition.x() << ", " << lastPosition.y() << std::endl;

    canvas.animate([&]() {

        chain.updateInverseKinematics(targetPosition, learningRate);
       // drawKinematicChain(chain, scene);
        camera->updateProjectionMatrix();
        renderer.render(*scene, *camera);



    });


    return 0;
}


