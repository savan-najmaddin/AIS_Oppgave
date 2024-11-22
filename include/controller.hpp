#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP


#include "Eigen/Core"
#include "Logikk.hpp"
#include "threepp/threepp.hpp"



class controller {

public:
    struct MyMouseListener : threepp::MouseListener {

        KinematicChain &chain;
        float &t;
        threepp::Canvas &canvas;
        threepp::OrthographicCamera &camera;


        MyMouseListener(float &t, KinematicChain &chain, threepp::Canvas &canvas, threepp::OrthographicCamera &camera)
            : t(t), chain(chain), canvas(canvas), camera(camera) {}

        void onMouseDown(int button, const threepp::Vector2 &pos) override {
            // adjusting the mouse position relative to the canvas size, PS adjusted for square canvas
            threepp::WindowSize const windowSize = canvas.size();


            float ndcX = (2.0f * pos.x) / windowSize.height() - 1.0f;
            float ndcY = 1.0f - (2.0f * pos.y) / windowSize.height();

            // for targetposition
            threepp::Vector3 ndcCoords(ndcX, ndcY, 0.0f);// Z = 0 for the XY-plane
            threepp::Vector3 worldCoords = ndcCoords.unproject(camera);


            Eigen::Vector2f target(worldCoords.x, worldCoords.y);
            chain.targetPosition(target);

        }
    };

};


#endif//CONTROLLER_HPP
