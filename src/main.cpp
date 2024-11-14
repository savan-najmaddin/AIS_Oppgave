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

    kinematicChain chain;
    chain.addJoint(Joint(M_PI , 5.0f));
    chain.addJoint(Joint(M_PI , 5.0f));
    chain.addJoint(Joint(M_PI , 5.0f));

    std::vector<std::shared_ptr<Object3D>> jointVisuals;
    for(size_t i = 0; i < chain.numJoints; ++i) {
        auto jointVisual = std::make_shared<Object3D>();
        auto link = std::make_shared<JointVisual>(chain.joints[i].length)->getMesh();
        jointVisual->add(link);
        jointVisuals.push_back(jointVisual);
    }

    for (auto& jointVisual : jointVisuals) {
        scene->add(jointVisual);
    }

    Eigen::Vector2f targetPosition = {2.0f, 3.0f};
    auto targetGeometry = SphereGeometry::create(0.5f, 16, 16);
    auto targetMaterial = MeshBasicMaterial::create({{"color", Color::green}});
    auto targetMesh = Mesh::create(targetGeometry, targetMaterial);
    targetMesh->position.set(targetPosition.x(), targetPosition.y(), 0);
    scene->add(targetMesh);


    std::cout << "Joint Positions:\n";
    for (const auto& joint : chain.joints) {
        std::cout << "Angle: " << joint.angle << ", Length: " << joint.length << std::endl;
    }

    Eigen::Vector2f lastPosition = chain.findEffectorPosition();
    std::cout<< "Last position: " << lastPosition.x() << ", " << lastPosition.y() << std::endl;



    canvas.animate([&]() {

        float dt = clock.getDelta();

        chain.updateInverseKinematics(targetPosition, 0.001f);

        float cumulativeAngle = 0.0f;
        Eigen::Vector2f position(0.0f, 0.0f);

        for(size_t i = 0; i < chain.numJoints; ++i) {
            cumulativeAngle += chain.joints[i].angle;

            jointVisuals[i]->position.set(position.x(), position.y(), 0);
            jointVisuals[i]->rotation.set(0, 0, cumulativeAngle);

            position.x() += chain.joints[i].length * std::cos(cumulativeAngle);
            position.y() += chain.joints[i].length * std::sin(cumulativeAngle);

        }

        Eigen::Vector2f posisjon = chain.findEffectorPosition();
        std::cout << "Effector position: " << posisjon.x() << ", " << posisjon.y() << std::endl;

        renderer.render(*scene, *camera);



    });


    return 0;
}


