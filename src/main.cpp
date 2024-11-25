#include "Eigen/Core"
#include "Logikk.hpp"
#include "controller.hpp"
#include "minScene.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"


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

        if (!initializeChain) {
            ImGui::InputInt("Number of joints: (1-10)", &numJoints, 1, 10);
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
    MyUI ui(canvas);

    int numJoints = ui.numJoints;
    float jointLength = ui.jointLength;
    float &learningRate = ui.learningRate;

    for (size_t i = 0; i < numJoints; ++i) {
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

    int frameCount{0};//Potential overflow after x hours.


    canvas.animate([&]() {
        frameCount += 1;//for å holde tid

        Eigen::Vector2f targetPosition = chain.getTargetPosition();

        targetMesh->position.set(targetPosition.x(), targetPosition.y(), 0);

        float cumulativeAngle = 0.0f;
        Eigen::Vector2f position(0.0f, 0.0f);

        if(ui.initializeChain)
        {
        chain.updateInverseKinematics(targetPosition, learningRate);
        chain.numJoints = numJoints;

        for (size_t i = 0; i < chain.numJoints; ++i) {
            cumulativeAngle += chain.joints[i].angle;

            jointVisuals[i]->position.set(position.x(), position.y(), 0);
            jointVisuals[i]->rotation.set(0, 0, cumulativeAngle);

            position.x() += chain.joints[i].length * std::cos(cumulativeAngle);
            position.y() += chain.joints[i].length * std::sin(cumulativeAngle);
        }


        renderer.render(*scene, *camera);

    }
        ui.render();
    });

    return 0;
}
