#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
#include "Eigen/Core"
#include "Logikk.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include <Visual.hpp>


using namespace threepp;

int main() {

    kinematicChain chain;

    auto parameter = canvasParameter();
    Canvas canvas(parameter);

    Clock clock;
    int frameCount{0}; //Potential overflow after x hours.

    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamera();



    controller::MyMouseListener ml{clock.elapsedTime, chain, canvas, *camera};
    canvas.addMouseListener(ml);


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


    auto targetGeometry = SphereGeometry::create(0.5f, 16, 16);
    auto targetMaterial = MeshBasicMaterial::create({{"color", Color::green}});
    auto targetMesh = Mesh::create(targetGeometry, targetMaterial);

    scene->add(targetMesh);


    std::cout << "Joint Positions:\n";
    for (const auto& joint : chain.joints) {
        std::cout << "Angle: " << joint.angle << ", Length: " << joint.length << std::endl;
    }

    Eigen::Vector2f lastPosition = chain.findEffectorPosition();
    std::cout<< "Last position: " << lastPosition.x() << ", " << lastPosition.y() << std::endl;



    canvas.animate([&]() {

        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetMesh->position.set(targetPosition.x(), targetPosition.y(), 0);

        frameCount += 1;

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


        Eigen::Vector2f effectorPosition = chain.findEffectorPosition();
        if(frameCount % 150 == 0){
            std::cout << "Effector position: " << effectorPosition.x() << ", " << effectorPosition.y() << std::endl;
            std::cout << "Effector actual position: " << effectorPosition.norm() << std::endl;
            std::cout << "Target postion: " << targetPosition.x() << ", " << targetPosition.y() << std::endl;
            std::cout << "Visual target position" << targetMesh->position.x << ", " << targetMesh->position.y << std::endl;

        }
        renderer.render(*scene, *camera);




    });


    return 0;
}


