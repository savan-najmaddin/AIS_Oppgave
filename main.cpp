#include "Logic.hpp"

#include "include/ImGui.hpp"
#include "include/controller.hpp"
#include "include/minScene.hpp"

#include "Visual.hpp"

#include <Objects.hpp>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//sette exceptions


using namespace threepp;


int main() {
    auto parameter = canvasParameter();
    Canvas canvas(parameter);
    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamera();

    IOCapture capture{};
    capture.preventMouseEvent = [] {
        return ImGui::GetIO().WantCaptureMouse;
    };
    canvas.setIOCapture(&capture);

    KinematicChain chain;
    MyUI ui(canvas);

    float &learningRate = ui.learningRate;

    Clock clock;
    controller::MyMouseListener ml{clock.elapsedTime, chain, canvas, *camera};
    canvas.addMouseListener(ml);

    VisualJoints visualJoints;

    SphereInitializer sphereInitializer;

    MySpheres centerCircle(0.5f, 32, 32, Color(0x0000FF)); //blå

    MySpheres targetCircle(0.5f, 32, 32, Color(0x800080)); //lilla

    MySpheres reachCircle(1.0f, 300, 300, Color(0x00AAAD)); //turkis
    reachCircle.material->transparent = true; //kan legges i konstruktør
    reachCircle.material->opacity = 0.2f;


    sphereInitializer.circleInitializer(scene, centerCircle); //for loop ?
    sphereInitializer.circleInitializer(scene, targetCircle);
    sphereInitializer.circleInitializer(scene, reachCircle);

    std::size_t prevNumJoints = 0;


    canvas.animate([&] {

        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetCircle.mesh->position.set(targetPosition.x(), targetPosition.y(), 0);

        if(ui.initializeChain)
        { //TODO flytt til egen metode, start her
            while (chain.joints.size() > ui.numJoints) {
                chain.removeJoint();
            }
            while (chain.joints.size() < ui.numJoints) {
                chain.addJoint(Joint(M_PI, ui.jointLength));
            }

            if (chain.joints.size() != prevNumJoints) {

                chain.updateMaxReach();
                visualJoints.setChain(*scene, chain);
                auto const geometry = SphereGeometry::create(chain.getMaxReach(), 64);
                reachCircle.mesh->setGeometry(geometry);
                prevNumJoints = chain.joints.size();
            }

            chain.updateInverseKinematics(targetPosition, learningRate);
            visualJoints.update(chain);
            //TODO flytt til egen metode, slutt her

            renderer.render(*scene, *camera);
        }
        ui.render();
    });

    return 0;
}
