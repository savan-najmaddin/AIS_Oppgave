#include "controller.hpp"
#include <iostream>

//legg til funksjoner til kontroller
controller::controller(Scene &scene) : scene(scene){};

void controller::onMouseDown(int key, const Vector2 &pos) {
    if (key == 0) {
        std::cout << "On mouse down" << std::endl;
    }
}

//legge til feil knapp advarsel?

void controller::onKeyPressed(KeyEvent evt) {
    if (evt.key == Key::D) {
        handleAKey();
    }
    else if (evt.key == Key::A) {
        handleSKey();
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

void controller::handleAKey() {
        //fyll inn
    }
void controller::handleSKey() {
        //
    }
void controller::handleEnterKey() {
        //
    }
void controller::wrongKeyEnter() {
        //
    }