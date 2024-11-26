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

    IOCapture capture{}; //kan slettes hvis jeg begrenser mus til maxreach
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

    MySpheres mySpheres;
    Spheres spheres;

    mySpheres.circleInitializer(scene);

    std::size_t prevNumJoints = 0;


    canvas.animate([&] {

        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        spheres.targetCircle->position.set(targetPosition.x(), targetPosition.y(), 0);

        if(ui.initializeChain)
        {
            while (chain.joints.size() > ui.numJoints) { //gpt std generate?, elr absoluttverider?
                chain.removeJoint();
            }
            while (chain.joints.size() < ui.numJoints) {
                chain.addJoint(Joint(M_PI, ui.jointLength));
            }

            if (chain.joints.size() != prevNumJoints) {

                chain.updateMaxReach();
                visualJoints.setChain(*scene, chain);
                auto const geometry = SphereGeometry::create(chain.getMaxReach(), 64);
                spheres.reachCircle->setGeometry(geometry);
                prevNumJoints = chain.joints.size();
            }

            chain.updateInverseKinematics(targetPosition, learningRate);
            visualJoints.update(chain);

            renderer.render(*scene, *camera);
        }
        ui.render();
    });

    return 0;
}
