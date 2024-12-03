#include "Logic.hpp"
#include "Visual.hpp"
#include "include/Controller.hpp"
#include "include/ImGui.hpp"
#include "include/MyScene.hpp"
#include <Handler.hpp>
#include <Objects.hpp>


int main() {
    auto parameter = canvasParameter();
    threepp::Canvas canvas(parameter);
    threepp::GLRenderer renderer(canvas.size());

    std::shared_ptr<threepp::Scene> scene = createScene();
    std::shared_ptr<threepp::OrthographicCamera> camera = createOrthographicCamera();

    KinematicChain chain;
    MyUI ui(canvas);
    VisualJoints visualJoints;

    float &learningRate = ui.learningRate;

    threepp::Clock clock;
    controller::MyMouseListener ml{clock.elapsedTime, chain, canvas, *camera};
    canvas.addMouseListener(ml);

    MySpheres centerCircle(0.35f, threepp::Color(0xffffff), scene);
    MySpheres targetCircle(0.35f, threepp::Color(0xffffff), scene);
    MySpheres reachCircle(1.0f, threepp::Color(0xffffff), scene, true, 0.2f);

    Handler handler;

    canvas.animate([&] {
        if (ui.clock) {
            chain.showTime(static_cast<KinematicChain::TimeUnit>(ui.timeUnit));
        }
        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetCircle.getMesh()->position.set(targetPosition.x(), targetPosition.y(), 0);

        if (ui.initializeChain) {
            handler.update(chain, ui, visualJoints, *scene, reachCircle, learningRate);
            renderer.render(*scene, *camera);
        }

        ui.render();
    });

    return 0;
}
