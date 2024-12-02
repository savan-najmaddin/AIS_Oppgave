#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP


#include "Logic.hpp"
#include "threepp/threepp.hpp"

/**
 * @brief this class is responsible for handling mouse events
 */

class controller {

public:
    struct MyMouseListener final : threepp::MouseListener {

        KinematicChain &chain;
        float &t;
        threepp::Canvas &canvas;
        threepp::OrthographicCamera &camera;

        MyMouseListener(float &t, KinematicChain &chain, threepp::Canvas &canvas, threepp::OrthographicCamera &camera);

        void onMouseDown(int button, const threepp::Vector2 &pos) override;
    };
};


#endif//CONTROLLER_HPP
