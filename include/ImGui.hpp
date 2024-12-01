#ifndef IMGUI_HPP
#define IMGUI_HPP


#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"


using namespace threepp;

class MyUI : public ImguiContext {//gjør om til klasse, ps wtf er final
public:
    explicit MyUI( Canvas &canvas);

    int numJoints;
    float jointLength;
    float learningRate;
    bool initializeChain;
    bool clock;
    bool dontClick;
    int timeUnit = 0;
    std::array<std::string, 3> timeUnits = {"Seconds", "Minutes", "Hours"};


    void onRender() override;

private:
    IOCapture m_capture;
    Canvas& m_canvas;
    int m_framecount{0};
    int m_countOfPress{0};
    std::exception e{};
};



#endif
