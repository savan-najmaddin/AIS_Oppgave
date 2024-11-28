#include "Logic.hpp"

#include "include/ImGui.hpp"
#include "include/controller.hpp"
#include "include/MyScene.hpp"

#include "Visual.hpp"

#include <Handler.hpp>
#include <Objects.hpp>



using namespace threepp;


int main() {
    auto parameter = canvasParameter(); //sett inn i game.run
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

    MySpheres centerCircle(0.5f, 32, 32, Color(0x0000FF)); //blå
    centerCircle.addToScene(scene);

    MySpheres targetCircle(0.5f, 32, 32, Color(0x800080)); //lilla
    targetCircle.addToScene(scene);

    MySpheres reachCircle(1.0f, 300, 300, Color(0x00AAAD), true, 0.2f); //turkis
    reachCircle.addToScene(scene);


    canvas.animate([&] {

        if (ui.randomPosition) {
            KinematicChain::circularMotion(chain.getTargetPosition(), chain.getMaxReach());
        }
        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetCircle.getMesh()->position.set(targetPosition.x(), targetPosition.y(), 0);

        if(ui.initializeChain)
         {
            Handler handler(chain, ui, visualJoints, *scene, reachCircle);

            chain.updateInverseKinematics(targetPosition, learningRate);
            visualJoints.updateJointVisual(chain);

            renderer.render(*scene, *camera);
        }

        ui.render();
    });

    return 0;
}
