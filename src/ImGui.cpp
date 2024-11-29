#include "ImGui.hpp"

using namespace threepp;

MyUI::MyUI(const Canvas &canvas)
    : ImguiContext(canvas.windowPtr()),
      numJoints(3),
      jointLength(5.0f),
      learningRate(0.3f),
      initializeChain(false),
      randomPosition(false){}


void MyUI::onRender() {

    ImGui::SetNextWindowPos({}, 0, {});
    ImGui::SetNextWindowSize({}, 0);
    ImGui::Begin("Bendern");

    ImGui::Text("Input field:");

    ImGui::InputInt("Number of joints: (1-10)", &numJoints, 1, 10);//tror maks 20
    if (!initializeChain) {
        ImGui::InputFloat("Joint Length (0.1 - 10.0):", &jointLength, 0.1f, 10.0f);

        if (ImGui::Button("Initialize Chain")) {
            initializeChain = true;
        }
    } else {
        ImGui::SliderFloat("Learning Rate:", &learningRate, 0.03f, 0.5f);
        ImGui::Checkbox("Circulare motion", &randomPosition);
    }
    ImGui::End();
}
