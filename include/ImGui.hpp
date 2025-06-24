/**
 * @brief this class is responsible for the user interface
 */

#ifndef IMGUI_HPP
#define IMGUI_HPP

#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"


class MyUI final : public ImguiContext {
public:
    explicit MyUI(threepp::Canvas &canvas);

    int numJoints;
    float jointLength;
    float learningRate;
    bool initializeChain;
    mutable bool dontClick;
    void onRender() override;

private:
    threepp::IOCapture m_capture;
    threepp::Canvas &m_canvas;
};


#endif
