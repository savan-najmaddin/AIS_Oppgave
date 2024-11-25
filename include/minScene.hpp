
#ifndef MINSCENE_HPP
#define MINSCENE_HPP

#include "threepp/threepp.hpp"



using namespace threepp;
struct CanvasParameters {
    float width;
    float height;
};

inline Canvas::Parameters canvasParameter(){ //vinduet til programmet
    Canvas::Parameters parameter;
    parameter.title("Min tittel");
    parameter.size(800.0f, 800.0f);
    parameter.vsync(true);
    parameter.resizable(false);
    return parameter;

};

inline auto createScene() {
    return Scene::create();
}

inline std::shared_ptr<OrthographicCamera> createOrthographicCamera() {
    float viewSize = 20.0f;
    float aspectRatio = 1.0f;

    auto camera = OrthographicCamera::create(
        -20, 20,
        20, -20, //aspectrartio er 1
        -100.0f, 100.0f
    );

    return camera;
}

class JointVisual {
public:
    JointVisual(float length, float width = 0.2f) {
        // Create geometry for the joint (a thin box)
        auto geometry = BoxGeometry::create(length, width, width);
        auto material = MeshBasicMaterial::create({{"color", Color::red}});

        mesh = Mesh::create(geometry, material);
        mesh->position.set(length / 2.0f , 0, 0); // Center the mesh
    }

    std::shared_ptr<Mesh> getMesh() {
        return mesh;
    }

private:
    std::shared_ptr<Mesh> mesh;
};


class myBox {
public:
    void boxSegment (float width, float height, float depth) {
        auto geometry = BoxGeometry::create(width, height, depth);
        auto material = MeshBasicMaterial::create();

        segment = Mesh::create(geometry, material);
    }
    std::shared_ptr<Mesh> getMesh () {
        return segment;
    }

private:
    std::shared_ptr<Mesh> segment;

};

//

#endif //MINSCENE_HPP
