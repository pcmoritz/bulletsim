#include <iostream>
#include <vector>
#include "FEMRegistration.hpp"
#include "SceneGeometry.hpp"

int main() {
	SceneGeometry cube = load("/home/pcm/Dropbox/data/cube.off");
	// SceneGeometry first = load("/home/pcm/Dropbox/data/standard-geometry-cloth-1.off");
	// SceneGeometry second = load("/home/pcm/Dropbox/data/standard-geometry-cloth-2.off");

	SceneGeometry first = load("/home/pcm/geometry1.off");
	SceneGeometry second = load("/home/pcm/geometry2.off");

	first.set_center(btVector3(3, -2, 16.0));
	second.set_center(btVector3(-3, -2, 16.0));

	/*
	std::vector<btVector3> vs = cube.get_vertices();
	for (int i = 0; i < vs.size(); i++) {
	  btVector3 temp = vs[i];
	  //std::cout << temp.getX() << " " << temp.getY() << " " << temp.getZ() << std::endl;
	}
	
	std::cout << "done reading edges" << std::endl;

	std::vector<Face> fs = cube.get_faces();
	for (int i = 0; i < fs.size(); i++) {
	  Face temp = fs[i];
	  //std::cout << temp.corners[0] << " " << temp.corners[1] << " " << temp.corners[2] << " " << temp.corners[3] << std::endl;
	}

	std::cout << "done reading faces" << std::endl;

	std::cout << on_boundary(btVector3(1.0, 1.1, 0.0), cube) << std::endl;
	std::cout << on_boundary(btVector3(1.0, 0.0, 0.0), cube) << std::endl;
	std::cout << on_boundary(btVector3(2.0, 0.0, 0.0), cube) << std::endl;
	std::cout << on_boundary(btVector3(-1.0, -1.0, -1.0), cube) << std::endl;
	*/

	tetgenio tet = constructMesh(first, second, -15.0, -15.0, 5.0, 15.0, 15.0, 30.0);
}
