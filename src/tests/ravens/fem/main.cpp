#include <iostream>
#include <vector>
#include "FEMRegistration.hpp"
#include "SceneGeometry.hpp"
#include "bullet.hpp"

int main() {
	// SceneGeometry cube = load("/home/pcm/Dropbox/data/cube.off");
	// SceneGeometry first = load("/home/pcm/Dropbox/data/standard-geometry-cloth-1.off");
	// SceneGeometry second = load("/home/pcm/Dropbox/data/standard-geometry-cloth-2.off");

	double xmin = -15.0;
	double ymin = -15.0;
	double zmin = 5.0;
	double xmax = 15.0;
	double ymax = 15.0;
	double zmax = 30.0;

	SceneGeometry a = load("/home/pcm/a.off");
	SceneGeometry b = load("/home/pcm/b.off");

	tetwrap::surface A = make_surface(2, a);
	tetwrap::surface B = make_surface(2, b);

	tetwrap::surface outer;
	// bottom
	outer.add_vertex(tetwrap::point(xmin, ymin, zmin));
	outer.add_vertex(tetwrap::point(xmax, ymin, zmin));
	outer.add_vertex(tetwrap::point(xmax, ymax, zmin));
	outer.add_vertex(tetwrap::point(xmin, ymax, zmin));
	// top
	outer.add_vertex(tetwrap::point(xmin, ymin, zmax));
	outer.add_vertex(tetwrap::point(xmax, ymin, zmax));
	outer.add_vertex(tetwrap::point(xmax, ymax, zmax));
	outer.add_vertex(tetwrap::point(xmin, ymax, zmax));
	//facets
	outer.add_facet(tetwrap::make_tetragon(0, 1, 2, 3)); // bottom
	outer.add_facet(tetwrap::make_tetragon(4, 5, 6, 7)); // top
	outer.add_facet(tetwrap::make_tetragon(0, 3, 7, 4)); // left
	outer.add_facet(tetwrap::make_tetragon(1, 5, 6, 2)); // right
	outer.add_facet(tetwrap::make_tetragon(0, 1, 5, 4)); // front
	outer.add_facet(tetwrap::make_tetragon(3, 2, 6, 7)); // back

	A.marker(2);
	B.marker(3);
	outer.marker(4);

	tetwrap::geometry scene;
	scene.add_component(A);
	scene.add_component(B);
	// scene.add_component(outer);

	tetgenio in = tetwrap::generate_input(scene);

	tetgenio out;
	char flags[] = "pq1.414a0.1";
	tetrahedralize(flags, &in, &out);

	std::cout << tetwrap::get_interior_point(A) << std::endl;

	btVector3 p(1, 1, 15);
	Shape s(A, xmin, ymin, zmin, xmax, ymax, zmax);
	std::cout << s.is_inside(p) << std::endl;

	//SceneGeometry first = load("/home/pcm/geometry1.off");
	//SceneGeometry second = load("/home/pcm/geometry2.off");

	// first.set_center(btVector3(3, -2, 16.0));
	// second.set_center(btVector3(-3, -2, 16.0));

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

	// tetgenio tet = constructMesh(first, second, -15.0, -15.0, 5.0, 15.0, 15.0, 30.0);
}
