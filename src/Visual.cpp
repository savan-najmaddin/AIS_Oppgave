#include "Visual.hpp"

void VisualJoints::setChain(threepp::Scene &scene, const KinematicChain &chain) {
    while (chain.getJoints().size() != visualJoints.size()) {
        while (chain.getJoints().size() < visualJoints.size()) {
            removeJoint(scene, chain);
        }
        while (chain.getJoints().size() > visualJoints.size()) {
            addJoint(scene, chain);
        }
    }
}

void VisualJoints::updateJointVisual(const KinematicChain &chain) const {
    float cumulativeAngle = 0.0f;
    Eigen::Vector2f position(0.0f, 0.0f);

    for (size_t i = 0; i < chain.getJoints().size(); ++i) {
        cumulativeAngle += chain.getJoints()[i].angle;

        const auto length = chain.getJoints()[i].length;
        visualJoints[i]->position.set(position.x() + length * std::cos(cumulativeAngle) / 2,
                                      position.y() + length * std::sin(cumulativeAngle) / 2, 0);
        visualJoints[i]->rotation.set(0, 0, cumulativeAngle);

        position.x() += chain.getJoints()[i].length * std::cos(cumulativeAngle);
        position.y() += chain.getJoints()[i].length * std::sin(cumulativeAngle);
    }
}

void VisualJoints::addJoint(threepp::Scene &scene, const KinematicChain &chain) {
    auto geometry = threepp::BoxGeometry::create(chain.getJoints().back().length, m_width, m_width);
    auto material = threepp::MeshBasicMaterial::create({{"color", threepp::Color::red}});
    auto mesh = std::make_shared<threepp::Mesh>(geometry, material);
    visualJoints.push_back(std::move(mesh));
    scene.add(*visualJoints.back());
}

void VisualJoints::removeJoint(threepp::Scene &scene, const KinematicChain &chain) {
    scene.remove(*visualJoints.back());
    visualJoints.pop_back();
    if (!chain.getJoints().empty()) {
        scene.add(*visualJoints.back());
    }
}

