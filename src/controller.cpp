#include "controller.hpp"
#include <iostream>

//legg til funksjoner til kontroller
controller::controller(Scene &scene, std::vector<armSegment> &armVec) : scene(scene), armVec(armVec){};

void controller::onMouseDown(int key, const Vector2 &pos) {
    if (key == 0) {
        Vector2 targetPosition = lastMousePosition(pos.x, pos.y);
    }

}

//legge til feil knapp advarsel?



void controller::onKeyPressed(KeyEvent evt) {
    if (evt.key == Key::D) {
        handleDKey();
    }
    else if (evt.key == Key::A) {
        handleAKey();
    }
    else if (evt.key == Key::ENTER) {
        handleEnterKey();
    }
    else {
        wrongKeyEnter();
    }
};


    void controller::onKeyReleased(KeyEvent evt) {
        if (evt.key == Key::D) {
            std::cout << "D key released" << std::endl;
        }

        if (evt.key == Key::A) {
            std::cout << "A key released" << std::endl;
        }
    }





