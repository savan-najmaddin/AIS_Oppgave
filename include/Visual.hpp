#ifndef VISUAL_HPP
#define  VISUAL_HPP

class Visualizer {
public:
  Visualizer(const std::shared_ptr<threepp::Scene>& scene)
      : scene_(scene) {
    // Initialize the line object and add it to the scene
    auto material = threepp::LineBasicMaterial::create();
    material->color = threepp::Color(0x000000); // Black color for the lines

    geometry_ = threepp::BufferGeometry::create();
    line_ = threepp::Line::create(geometry_, material);
    scene_->add(line_);
  }

  void drawKinematicChain(const kinematicChain& chain) {
    // Calculate joint positions
    std::vector<float> vertices;
    Eigen::Vector2f position(0.0f, 0.0f);
    float cumulativeAngle = 0.0f;

    // Add starting point
    vertices.push_back(position.x());
    vertices.push_back(position.y());
    vertices.push_back(0.0f); // z-coordinate remains 0 for 2D

    for (const auto& joint : chain.joints) {
      cumulativeAngle += joint.angle;
      Eigen::Vector2f nextPosition;
      nextPosition.x() = position.x() + joint.length * std::cos(cumulativeAngle);
      nextPosition.y() = position.y() + joint.length * std::sin(cumulativeAngle);

      // Add next joint position
      vertices.push_back(nextPosition.x());
      vertices.push_back(nextPosition.y());
      vertices.push_back(0.0f); // z-coordinate remains 0 for 2D

      position = nextPosition;
    }

    // Update geometry's vertices
    geometry_->setAttribute("position", threepp::FloatBufferAttribute::create(vertices, 3));
    geometry_->computeBoundingSphere();

    // Optional: Update the line's material or other properties if needed
  }

private:
  std::shared_ptr<threepp::Scene> scene_;
  std::shared_ptr<threepp::BufferGeometry> geometry_;
  std::shared_ptr<threepp::Line> line_;
};

#endif
