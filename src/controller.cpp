#include "controller.hpp"
#include <iostream>

//legg til funksjoner til kontroller
 controller::controller(Scene &scene) : scene(scene) {};

    void controller::onMouseDown(int key, const Vector2 &pos) {
        if (key == 0) {
            std::cout << "On mouse down" << std::endl;
        }
    }

    //legge til feil knapp advarsel?

    void controller::onKeyPressed(KeyEvent evt) {
        if (evt.key == Key::D) {
            std::cout << "Knapp ' d ' trykkes "
        }

        if (evt.key == Key::A) {
            handleSKey();
        }
    }