#ifndef OBJEKT_HPP
#define OBJEKT_HPP

#include "threepp/threepp.hpp"


using namespace threepp;


class interactivPlane {
public:

  interactivPlane(float width, float height) {
    auto geometry = PlaneGeometry::create(width, height);

    auto material = MeshBasicMaterial::create();
    material->color = Color::yellow;

    plane = Mesh::create(geometry, material);
  }

  std::shared_ptr<Mesh> getPlane() {
    return plane;

  }

private:
  std::shared_ptr<Mesh> plane;
};


class armSegment {
public:
  //fyi husk å differensier mellom logikk og grafikk
  armSegment(float width, float height, float depth){

    auto geometry = BoxGeometry::create(width, height, depth);

    auto material= MeshBasicMaterial::create();


    segment = Mesh::create(geometry, material);

  }
  //skriv en funksjon som returnerer segment
  std::shared_ptr<Mesh> getSegment() {
    return segment;
  }

private:
  std::shared_ptr<Mesh> segment;
};








#endif //OBJEKT_HPP
