#include "FEMRegistration.hpp"
#include "tetgen/tetgen.h"
#include "mesh.hpp"

NonRigidTransform FEMRegistration::constructTransform()
{
	return NonRigidTransform();
}

dolfin::Mesh constructMesh(const SceneGeometry& a, const SceneGeometry& b,
		double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
{
	std::vector<btVector3> a_vert = a.get_vertices();
	std::vector<btVector3> b_vert = b.get_vertices();
	tetgenio in;
	// All indices start from 1
	in.firstnumber = 1;
	// The 4 is for the vertices of the outer boundary
	in.numberofpoints = a_vert.size() + b_vert.size() + 4;
	in.pointlist = new REAL[in.numberofpoints * 3];
	for(int i = 0; i < a_vert.size(); i++) {
		in.pointlist[3 * i] = a_vert[i].getX();
		in.pointlist[3 * i + 1] = a_vert[i].getY();
		in.pointlist[3 * i + 2] = a_vert[i].getZ();
	}
	int V = a_vert.size();
	for(int i = 0; i < b_vert.size(); i++) {
		in.pointlist[V + 3 * i] = b_vert[i].getX();
		in.pointlist[V + 3 * i + 1] = b_vert[i].getY();
		in.pointlist[V + 3 * i + 2] = b_vert[i].getZ();
	}
	// The last few vertices are the ones of the outer boundary
	V = a_vert.size() + b_vert.size();
	// First point:
	in.pointlist[V] = xmin;
	in.pointlist[V + 1] = ymin;
	in.pointlist[V + 2] = zmin;
	// Second point:
	in.pointlist[V + 3] = xmax;
	in.pointlist[V + 4] = ymin;
	in.pointlist[V + 5] = zmin;
	// Third point:
	in.pointlist[V + 6] = xmax;
	in.pointlist[V + 7] = ymax;
	in.pointlist[V + 8] = zmin;
	// Fourth point:
	in.pointlist[V + 9] = xmin;
	in.pointlist[V + 10] = ymax;
	in.pointlist[V + 11] = zmin;
	// Fifth point:
}

Meshes FEMRegistration::constructMesh(const SceneGeometry& cloth1, const SceneGeometry& cloth2)
{
	tetgenio io;

	Meshes meshes;
	dolfin::csg::Exact_Polyhedron_3 standard_cloth_1 = load_off_file("/home/pcm/data/standard-geometry-cloth-1.off");
	dolfin::csg::Exact_Polyhedron_3 standard_cloth_2 = load_off_file("/home/pcm/data/standard-geometry-cloth-2.off");
	dolfin::csg::Exact_Polyhedron_3 cube = load_off_file("/home/pcm/data/cube.off");

	std::cout << "after load" << std::endl;
	std::cout << standard_cloth_1 << std::endl;
	std::cout << "valid?" << std::endl;
	std::cout << standard_cloth_1.is_valid() << std::endl;
	std::cout << standard_cloth_2 << std::endl;
	std::cout << "valid?" << std::endl;
	std::cout << standard_cloth_2.is_valid() << std::endl;
	std::cout << "after load" << std::endl;

	double cell_size = 1.0;
	bool detect_sharp_features = true;

	// scale the outer box
	double xmid = (xmin + xmax)/2;
	double ymid = (ymin + ymax)/2;
	double zmid = (zmin + zmax)/2;

	std::cout << xmid << std::endl;
	std::cout << ymid << std::endl;
	std::cout << zmid << std::endl;

	std::cout << "point 1" << std::endl;

	//CGAL::Aff_transformation_3<dolfin::csg::Exact_Kernel> outer_scale(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
	CGAL::Aff_transformation_3<dolfin::csg::Exact_Kernel> outer_scale(xmax - xmid, 0, 0, 0, 0, ymax - ymid, 0, 0, 0, 0, zmax - zmid, 0);
	CGAL::Aff_transformation_3<dolfin::csg::Exact_Kernel> outer_trans(1, 0, 0, xmid, 0, 1, 0, ymid, 0, 0, 1, zmid);
	std::transform(cube.points_begin(), cube.points_end(), cube.points_begin(), outer_trans * outer_scale);
	//std::transform(cube.points_begin(), cube.points_end(), cube.points_begin(), outer_scale);

	std::cout << "point 2" << std::endl;

	dolfin::csg::Nef_polyhedron_3 Omega(cube);

	std::cout << "point 3" << std::endl;

	Omega -= standard_cloth_1;
	std::cout << standard_cloth_1 << std::endl;
	Omega -= standard_cloth_2;
	std::cout << standard_cloth_2 << std::endl;

	std::cout << "point 4" << std::endl;

	dolfin::csg::Polyhedron_3 poly = nef_to_poly(Omega);
	std::cout << "point 5" << std::endl;
	generate(*meshes.domain_mesh, poly, cell_size);

	std::cout << "point 6" << std::endl;

	dolfin::plot(*meshes.domain_mesh, "mesh of the domain");
	std::cout << "point 7" << std::endl;
	dolfin::interactive(true);

	std::cout << "point 8" << std::endl;

	return meshes;
}
