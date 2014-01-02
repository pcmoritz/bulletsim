#include "FEMRegistration.hpp"

dolfin::csg::Exact_Polyhedron_3 load_off_file(const char* file_name)
{
	dolfin::csg::Exact_Polyhedron_3 result;
	std::cout << "reading file " << file_name << std::endl;
	std::ifstream file(file_name);
	file >> result;
	return result;
}

dolfin::csg::Polyhedron_3 nef_to_poly(const dolfin::csg::Nef_polyhedron_3& nef)
{
	dolfin::csg::Exact_Polyhedron_3 exact_poly;
	nef.convert_to_polyhedron(exact_poly);
	dolfin::csg::Polyhedron_3 result;
	copy_to(exact_poly, result);
	return result;
}

Transform FEMRegistration::constructTransform()
{
	return Transform();
}

Meshes FEMRegistration::constructMesh(const SceneGeometry& cloth1, const SceneGeometry& cloth2)
{
	Meshes meshes;
	dolfin::csg::Exact_Polyhedron_3 standard_cloth_1 = load_off_file("standard-geometry-cloth-1.off");
	dolfin::csg::Exact_Polyhedron_3 standard_cloth_2 = load_off_file("standard-geometry-cloth-2.off");
	dolfin::csg::Exact_Polyhedron_3 cube = load_off_file("cube.off");

	double cell_size = 1.0;
	bool detect_sharp_features = true;

	// scale the outer box
	double xmid = (xmin + xmax)/2;
	double ymid = (ymin + ymax)/2;
	double zmid = (zmin + zmax)/2;

	CGAL::Aff_transformation_3<dolfin::csg::Exact_Kernel> outer_scale(xmax - xmid, 0, 0, 0, 0, ymax - ymid, 0, 0, 0, 0, zmax - zmid, 0);
	CGAL::Aff_transformation_3<dolfin::csg::Exact_Kernel> outer_trans(1, 0, 0, xmid, 0, 1, 0, ymid, 0, 0, 1, zmid);
	std::transform(cube.points_begin(), cube.points_end(), cube.points_begin(), outer_trans * outer_scale);

	dolfin::csg::Nef_polyhedron_3 Omega(cube);

	Omega -= standard_cloth_1;
	Omega -= standard_cloth_2;

	dolfin::csg::Polyhedron_3 poly = nef_to_poly(Omega);
	dolfin::Mesh domain_mesh;
	generate(domain_mesh, poly, cell_size);

	meshes.domain_mesh = domain_mesh;

	dolfin::plot(domain_mesh, "mesh of the domain");
	dolfin::interactive(true);

	return meshes;
}
