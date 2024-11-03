#ifndef OBJEKT_HPP
#define OBJEKT_HPP

#include "threepp/threepp.hpp"
//inkluder eigen header

using namespace threepp;

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
