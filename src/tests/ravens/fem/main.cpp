#include <iostream>
#include "FEMRegistration.hpp"
#include "SceneGeometry.hpp"

int main() {
	SceneGeometry cube = load("/home/pcm/data/cube.off");
	SceneGeometry first = load("/home/pcm/data/standard-geometry-cloth-1.off");
	SceneGeometry second = load("/home/pcm/data/standard-geometry-cloth-2.off");
	std::cout << on_boundary(btVector3(1.0, 1.1, 0.0), cube) << std::endl;
	std::cout << on_boundary(btVector3(1.0, 0.0, 0.0), cube) << std::endl;
	std::cout << on_boundary(btVector3(2.0, 0.0, 0.0), cube) << std::endl;
	std::cout << on_boundary(btVector3(-1.0, -1.0, -1.0), cube) << std::endl;
}
