#include "Eigen/Core"
#include "Logikk.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"
#include "ImGui.hpp"

#include <Visual.hpp>


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
    capture.preventKeyboardEvent = [] {
        return ImGui::GetIO().WantCaptureKeyboard;
    };
    canvas.setIOCapture(&capture);

    KinematicChain chain;
    MyUI ui(canvas);

    float &learningRate = ui.learningRate;

    Clock clock;
    controller::MyMouseListener ml{clock.elapsedTime, chain, canvas, *camera};
    canvas.addMouseListener(ml);

    VisualJoints visualJoints;

    auto targetCircle = MySpheres::createSphere(0.5f, 32, 32, Color(0x800080)); //lilla

    auto centerCircle = MySpheres::createSphere(0.5f, 32, 32, Color(0xffffff)); //blå

    auto reachCircle = MySpheres::createSphere(1.0f, 300, 300, Color(0x00AAAD)); //turkis
    reachCircle->material()->transparent = true;
    reachCircle->material()->opacity = 0.2f;

    scene->add(targetCircle);
    scene->add(centerCircle);
    scene->add(reachCircle);

    std::size_t prevNumJoints = 0;


    canvas.animate([&] {

        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetCircle->position.set(targetPosition.x(), targetPosition.y(), 0);

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
                reachCircle->setGeometry(geometry);
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
