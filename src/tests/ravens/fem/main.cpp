#include <iostream>
#include <vector>
#include "FEMRegistration.hpp"
#include "SceneGeometry.hpp"

int main() {
	SceneGeometry cube = load("/home/pcm/data/cube.off");
	//SceneGeometry first = load("/home/pcm/data/standard-geometry-cloth-1.off");
	//SceneGeometry second = load("/home/pcm/data/standard-geometry-cloth-2.off");
	
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
}
