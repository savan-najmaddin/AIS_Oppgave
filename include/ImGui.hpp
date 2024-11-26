#ifndef IMGUI_HPP
#define IMGUI_HPP


#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"


using namespace threepp;

class MyUI : public ImguiContext {//gjør om til klasse, ps wtf er final
public:
    explicit MyUI(const Canvas &canvas);

    int numJoints;
    float jointLength;
    float learningRate;
    bool initializeChain;


    void onRender() override;

};


#endif
