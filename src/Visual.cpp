#include "Visual.hpp"


void VisualJoints::setChain(threepp::Scene &scene, const KinematicChain &chain) {

    while (chain.getJoints().size() < joints.size()) {
        removeJoint(scene);
    }
    while (chain.getJoints().size() > joints.size()) {
        addJoint(scene, chain);
    }
}

void VisualJoints::updateJointVisual(const KinematicChain &chain) const {
    float cumulativeAngle = 0.0f;
    Eigen::Vector2f position(0.0f, 0.0f);

    for (size_t i = 0; i < chain.getJoints().size(); ++i) {
        cumulativeAngle += chain.getJoints()[i].angle;

        const auto length = chain.getJoints()[i].length;
        joints[i]->position.set(position.x() + length * std::cos(cumulativeAngle) / 2,
                                      position.y() + length * std::sin(cumulativeAngle) / 2, 0);
        joints[i]->rotation.set(0, 0, cumulativeAngle);

        position.x() += chain.getJoints()[i].length * std::cos(cumulativeAngle);
        position.y() += chain.getJoints()[i].length * std::sin(cumulativeAngle);
    }
}

void VisualJoints::addJoint(threepp::Scene &scene, const KinematicChain &chain) {
    auto geometry = threepp::BoxGeometry::create(chain.getJoints().back().length, m_width, m_width);
    auto material = threepp::MeshBasicMaterial::create({{"color", threepp::Color::red}});
    auto mesh = std::make_shared<threepp::Mesh>(geometry, material);
    joints.push_back(std::move(mesh));
    scene.add(*joints.back());
}

void VisualJoints::removeJoint(threepp::Scene &scene) {
    scene.remove(*joints.back());
    joints.pop_back();
}

