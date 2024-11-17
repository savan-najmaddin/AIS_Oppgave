#include "threepp/threepp.hpp"
//#include "threepp/extras/imgui/ImguiContext.hpp"
#include "Eigen/Core"
#include "Logikk.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include <Visual.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


using namespace threepp;

int main() {

    int numJoints{3};
    float jointLength{5};

    auto parameter = canvasParameter();
    Canvas canvas(parameter);
    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamera();

    KinematicChain chain;
    for (size_t i = 0; i < numJoints; i++) {
        chain.addJoint(Joint(M_PI, jointLength));
    }
    float maxReach = chain.getMaxReach(chain);

    Clock clock;
    controller::MyMouseListener ml{clock.elapsedTime, chain, canvas, *camera};
    canvas.addMouseListener(ml);

    std::vector<std::shared_ptr<Object3D>> jointVisuals;
    for (size_t i = 0; i < chain.numJoints; ++i) {
        auto jointVisual = std::make_shared<Object3D>();
        auto link = std::make_shared<JointVisual>(chain.joints[i].length)->getMesh();
        jointVisual->add(link);
        jointVisuals.push_back(jointVisual);
    }

    for (auto &jointVisual: jointVisuals) {
        scene->add(jointVisual);
    }

    auto targetGeometry = SphereGeometry::create(0.5f, 16, 16);
    auto targetMaterial = MeshBasicMaterial::create({{"color", Color::green}});
    auto targetMesh = Mesh::create(targetGeometry, targetMaterial);

    auto circleGeometry = SphereGeometry::create(maxReach, 64);
    auto circleMaterial = MeshBasicMaterial::create();
    circleMaterial->color = Color(0xffffff);
    circleMaterial->transparent = true;
    circleMaterial->opacity = 0.2f;

    auto circleMesh = Mesh::create(circleGeometry, circleMaterial);
    circleMesh->rotation.x = math::degToRad(-90);

    circleMesh->position.set(0, 0, -0.1);

    scene->add(targetMesh);
    scene->add(circleMesh);

    std::cout << "Joint Positions:\n";
    for (const auto &joint: chain.joints) {
        std::cout << "Angle: " << joint.angle << ", Length: " << joint.length << std::endl;
    }

    Eigen::Vector2f lastPosition = chain.findEffectorPosition();
    std::cout << "Last position: " << lastPosition.x() << ", " << lastPosition.y() << std::endl;

    int frameCount{0};//Potential overflow after x hours.

    canvas.animate([&]() {
        frameCount += 1;//for å holde tid

        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetMesh->position.set(targetPosition.x(), targetPosition.y(), 0);

        chain.updateInverseKinematics(targetPosition, 0.1f);

        float cumulativeAngle = 0.0f;
        Eigen::Vector2f position(0.0f, 0.0f);

        for (size_t i = 0; i < chain.numJoints; ++i) {
            cumulativeAngle += chain.joints[i].angle;

            jointVisuals[i]->position.set(position.x(), position.y(), 0);
            jointVisuals[i]->rotation.set(0, 0, cumulativeAngle);

            position.x() += chain.joints[i].length * std::cos(cumulativeAngle);
            position.y() += chain.joints[i].length * std::sin(cumulativeAngle);
        }

        Eigen::Vector2f effectorPosition = chain.findEffectorPosition();
        if (frameCount % 150 == 0) {
            std::cout << "Effector position: " << effectorPosition.x() << ", " << effectorPosition.y() << std::endl;
            std::cout << "Effector actual position: " << effectorPosition.norm() << std::endl;
            std::cout << "Target postion: " << targetPosition.x() << ", " << targetPosition.y() << std::endl;
            std::cout << "Visual target position" << targetMesh->position.x << ", " << targetMesh->position.y << std::endl;
        }
        renderer.render(*scene, *camera);
    });

    return 0;
}
