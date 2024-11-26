#include "Eigen/Core"
#include "Logikk.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"

#include <Visual.hpp>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//sette exceptions


using namespace threepp;


struct MyUI : public ImguiContext {//gjør om til klasse
    int numJoints;
    float jointLength;
    float learningRate;
    bool initializeChain;

    explicit MyUI(const Canvas &canvas)
        : ImguiContext(canvas.windowPtr()),
          numJoints(3),
          jointLength(5.0f),
          learningRate(0.08f),
          initializeChain(false) {}

    void onRender() override {
        ImGui::SetNextWindowPos({}, 0, {});
        ImGui::SetNextWindowSize({}, 0);
        ImGui::Begin("Bendern");

        ImGui::Text("Input field: ");

        ImGui::InputInt("Number of joints: (1-10)", &numJoints, 1, 10); //tror lim er 20

        if (!initializeChain) {
            ImGui::InputFloat("Joint Length (0.1 - 10.0): ", &jointLength, 0.1f, 10.0f);

            if (ImGui::Button("Initialize Chain: ")) {
                initializeChain = true;
            }
        } else {
            ImGui::SliderFloat("Learning Rate: ", &learningRate, 0.03f, 0.5f);

        }
        ImGui::End();
    }
};

using namespace threepp;

int main() {
    auto parameter = canvasParameter();
    Canvas canvas(parameter);
    GLRenderer renderer(canvas.size());

    std::shared_ptr<Scene> scene = createScene();
    std::shared_ptr<OrthographicCamera> camera = createOrthographicCamera();

    IOCapture capture{}; //er dette nødvendig?
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
    MySpheres sphere;

    auto targetCircle = sphere.createSphere(0.5f, 32, 32, Color(0x800080)); //lilla

    auto centerCircle = sphere.createSphere(0.5f, 32, 32, Color(0x0000ff)); //blå

    auto reachCircle = sphere.createSphere(1.0f, 300, 300, Color(0x00AAAD)); //turkis
    reachCircle->material()->transparent = true;
    reachCircle->material()->opacity = 0.2f;
    //reachCircleMesh->position.z = -0.1;

    scene->add(targetCircle);
    scene->add(centerCircle);
    scene->add(reachCircle);

    std::size_t prevNumJoints = 0;


    canvas.animate([&]() {

        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetCircle->position.set(targetPosition.x(), targetPosition.y(), 0);

        if(ui.initializeChain)
        {
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
