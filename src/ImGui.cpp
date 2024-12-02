#include "ImGui.hpp"
#include <iostream>


using namespace threepp;

MyUI::MyUI(Canvas &canvas)
    : ImguiContext(canvas.windowPtr()),
      m_canvas(canvas),
      numJoints(3),
      jointLength(5.0f),
      learningRate(0.01f),
      initializeChain(false),
      clock(false),
      dontClick(false) {
    m_capture.preventMouseEvent = [] { //from threepp
        return ImGui::GetIO().WantCaptureMouse;
    };
    m_canvas.setIOCapture(&m_capture);
}

void MyUI::onRender() {
    ImGui::SetNextWindowPos({}, 0, {});
    ImGui::SetNextWindowSize({}, 0);
    ImGui::Begin("Bendy ");

    ImGui::Text("Input field:");
    try {
        ImGui::InputInt("Number of joints: (1-10)", &numJoints, 1, 10);
        if (numJoints > 10 || numJoints < 0) {
            throw std::out_of_range("Input a number from 0 - 10");
        }
    } catch (const std::exception &) {
        numJoints = std::clamp(numJoints, 0, 10);
    }

    if (!initializeChain) {
        ImGui::InputFloat("Joint Length (0.1 - 10.0):", &jointLength, 0.1f, 10.0f);

        if (ImGui::Button("Initialize Chain")) {
            initializeChain = true;
        }
    } else {
        ImGui::SliderFloat("Learning Rate:", &learningRate, 0.0001f, 0.1f);
        ImGui::Checkbox("Show current time", &clock);
        ImGui::Checkbox("Don't click", &dontClick);
        if (clock) {
            ImGui::SliderInt("Clock unit", &timeUnit, 0, 2, timeUnits[timeUnit].c_str());
        }
    }
    ImGui::End();
}
