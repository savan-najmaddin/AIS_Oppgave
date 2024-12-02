#ifndef IMGUI_HPP
#define IMGUI_HPP


#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"



class MyUI final : public ImguiContext {//gjør om til klasse, ps wtf er final
public:
    explicit MyUI( threepp::Canvas &canvas);

    int numJoints;
    float jointLength;
    float learningRate;
    bool initializeChain;
    bool clock;
    mutable bool dontClick;
    int timeUnit = 0;
    std::array<std::string, 3> timeUnits = {"Seconds", "Minutes", "Hours"};


    void onRender() override;

private:
    threepp::IOCapture m_capture;
    threepp::Canvas& m_canvas;
    int m_framecount{0};
    int m_countOfPress{0};
    std::exception e{};
};



#endif
