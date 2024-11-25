#include "Visual.hpp"

void VisualJoints::setChain(threepp::Scene &scene, const KinematicChain &chain) {
    if (chain.joints.size() < joints.size()) {
        while (chain.joints.size() < joints.size()) {
            scene.remove(*joints.back());
            joints.pop_back();
        }
    } else {
        while (chain.joints.size() > joints.size()) {
            const std::size_t i = joints.size();
            auto geometry = threepp::BoxGeometry::create(chain.joints[i].length, WIDTH, WIDTH);
            auto material = threepp::MeshBasicMaterial::create({{"color", threepp::Color::red}});
            auto mesh = threepp::Mesh::create(geometry, material);
            mesh->position.set(chain.joints[i].length / 2.0f, 0, 0);
            joints.push_back(std::move(std::make_unique<threepp::Mesh>()));
            joints.back()->copy(*mesh);
            scene.add(*joints.back());
        }
    }
}
void VisualJoints::update(const KinematicChain &chain) {
    float cumulativeAngle = 0.0f;
    Eigen::Vector2f position(0.0f, 0.0f);
    for (size_t i = 0; i < chain.joints.size(); ++i) {
        cumulativeAngle += chain.joints[i].angle;

        joints[i]->position.set(position.x(), position.y(), 0);
        joints[i]->rotation.set(0, 0, cumulativeAngle);

        position.x() += chain.joints[i].length * std::cos(cumulativeAngle);
        position.y() += chain.joints[i].length * std::sin(cumulativeAngle);
    }
}