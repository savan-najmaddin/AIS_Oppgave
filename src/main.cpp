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

        ImGui::InputInt("Number of joints: (1-10)", &numJoints, 1, 10);
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

    IOCapture capture{};
    capture.preventMouseEvent = [] {
        return ImGui::GetIO().WantCaptureMouse;
    };
    capture.preventKeyboardEvent = [] {
        return ImGui::GetIO().WantCaptureKeyboard;
    };
    canvas.setIOCapture(&capture);

    KinematicChain chain;
    Joint joint;
    MyUI ui(canvas);

    float &learningRate = ui.learningRate;

    Clock clock;
    controller::MyMouseListener ml{clock.elapsedTime, chain, canvas, *camera};
    canvas.addMouseListener(ml);

    VisualJoints visualJoints;

    auto targetGeometry = SphereGeometry::create(0.5f, 16, 16);
    auto targetMaterial = MeshBasicMaterial::create({{"color", Color::green}});
    auto targetMesh = Mesh::create(targetGeometry, targetMaterial);

    auto circleGeometry = SphereGeometry::create(1, 64);
    auto circleMaterial = MeshBasicMaterial::create();
    circleMaterial->color = Color(0xffffff);
    circleMaterial->transparent = false;
    circleMaterial->opacity = 0.2f;

    auto reachGeometry = SphereGeometry::create(1, 64);
    auto reachMaterial = MeshBasicMaterial::create();
    reachMaterial->color = Color(0xffffff);
    reachMaterial->transparent = true;
    reachMaterial->opacity = 0.2f;
    auto reachCircleMesh = Mesh::create(reachGeometry, reachMaterial);
    reachCircleMesh->position.z = -0.1;

    auto circleMesh = Mesh::create(circleGeometry, circleMaterial);
    circleMesh->rotation.x = math::degToRad(-90);

    circleMesh->position.set(0, 0, 0.1);

    scene->add(targetMesh);
    scene->add(circleMesh);
    scene->add(reachCircleMesh);

    std::size_t prevNumJoints = 0;


    canvas.animate([&]() {

        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetMesh->position.set(targetPosition.x(), targetPosition.y(), 0);

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
                auto geometry = threepp::SphereGeometry::create(chain.getMaxReach(), 64);
                reachCircleMesh->setGeometry(geometry);
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
