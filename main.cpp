#include "Logic.hpp"

#include "include/ImGui.hpp"
#include "include/Controller.hpp"
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
    VisualJoints visualJoints;

    float &learningRate = ui.learningRate;

    Clock clock;
    controller::MyMouseListener ml{clock.elapsedTime, chain, canvas, *camera};
    canvas.addMouseListener(ml);

    MySpheres centerCircle(0.5f, Color(0x0000FF), scene );
    MySpheres targetCircle(0.5f, Color(0x800080), scene);
    MySpheres reachCircle(1.0f, Color(0x00AAAD), scene, true, 0.2f);


    canvas.animate([&] {

        if (ui.randomPosition) {
            KinematicChain::circularMotion(chain.getTargetPosition(), chain.getMaxReach());
        }
        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetCircle.getMesh()->position.set(targetPosition.x(), targetPosition.y(), 0);

        if(ui.initializeChain)
         {
            Handler handler(chain, ui, visualJoints, *scene, reachCircle, targetPosition, learningRate);
            renderer.render(*scene, *camera);
        }

        ui.render();
    });

    return 0;
}
