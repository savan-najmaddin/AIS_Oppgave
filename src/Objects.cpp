#include "Objects.hpp"





auto MySpheres::createSphere(float radius, int widthSegments, int heightSegments, auto color) {
    std::shared_ptr<threepp::SphereGeometry> geometry = threepp::SphereGeometry::create(radius, widthSegments, heightSegments);
    std::shared_ptr<threepp::MeshBasicMaterial> material = threepp::MeshBasicMaterial::create();
    material->color = color;
    std::shared_ptr<threepp::Mesh> mesh = threepp::Mesh::create(geometry, material);
    return mesh;
}

void MySpheres::sphereToScene(std::shared_ptr<threepp::Scene> scene, auto sphere) {
    scene->add(sphere);
}

Spheres MySpheres::circleInitializer(std::shared_ptr<threepp::Scene> scene)  {
    Spheres spheres;

    spheres.centerCircle = createSphere(1.0f, 32, 32, threepp::Color(0x00ffff));
    sphereToScene(scene, spheres.centerCircle);

    spheres.targetCircle = createSphere(1.0f, 32, 32, threepp::Color(0x00ff00));
    sphereToScene(scene, spheres.targetCircle);

    spheres.reachCircle = createSphere(1.0f, 32, 32, threepp::Color(0xff0000));
    spheres.reachCircle->material()->transparent = true;
    spheres.reachCircle->material()->opacity = 0.2f;
    sphereToScene(scene, spheres.reachCircle);

    return spheres;
}