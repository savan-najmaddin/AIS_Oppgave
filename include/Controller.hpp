/**
 * @brief this class is responsible for handling mouse events
 *
 * Mouse listner returns a value in threepp::Vector2, which is then converted to NDC coordinates, due to
 * the cordinates originally being world cordinates. later the ndc cordinates are put in Eigen::Vector2f form
 * and this is used to set the target position of the chain
 *
 * Sources: GPT was used to let me know that World Cordinates and NDC cordinates are a thing.
 *
 * @param int button, const threepp::Vector2 &pos
 */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "Logic.hpp"
#include "threepp/threepp.hpp"


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
