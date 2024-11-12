#ifndef VISUAL_HPP
#define  VISUAL_HPP

void inline drawKinematicChain(const kinematicChain& chain, const std::shared_ptr<threepp::Scene>& scene) {
  // Remove existing lines from the scene
  scene->children.clear();

  // Prepare geometry and material
  auto geometry = threepp::BufferGeometry::create();
  auto material = threepp::LineBasicMaterial::create();
  material->color = threepp::Color(0x000000); // Black color for the lines

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

  // Set geometry's vertices
  geometry->setAttribute("position", threepp::FloatBufferAttribute::create(vertices, 3));

  // Create the line and add it to the scene
  auto line = threepp::Line::create(geometry, material);
  scene->add(line);
}
#endif
