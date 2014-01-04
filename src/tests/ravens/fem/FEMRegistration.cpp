#include "FEMRegistration.hpp"
#include "SceneGeometry.hpp"
#include <dolfin/mesh/MeshEditor.h>
#include <fstream>

NonRigidTransform FEMRegistration::constructTransform()
{
	return NonRigidTransform();
}

tetwrap::facet make_facet(const Face& f) {
	tetwrap::facet face;
	face.add_vertex(f.corners[0]);
	face.add_vertex(f.corners[1]);
	face.add_vertex(f.corners[2]);
	face.add_vertex(f.corners[3]);
	return face;
}

tetwrap::surface make_surface(const SceneGeometry& g) {
	tetwrap::surface result;
	std::vector<btVector3> vert = g.get_vertices();
	std::vector<Face> face = g.get_faces();
	for(int i = 0; i < vert.size(); i++)
		result.add_vertex(tetwrap::point(vert[i].getX(), vert[i].getY(), vert[i].getZ()));
	for(int i = 0; i < face.size(); i++)
		result.add_facet(make_facet(face[i]));
	return result;
}

tetwrap::surface make_outer_surface(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax) {
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

	return outer;
}

tetgenio constructMesh(const SceneGeometry& a, const SceneGeometry& b,
		double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
{
	std::cout << "start constructing the mesh" << std::endl;

	tetwrap::surface A = make_surface(a);
	tetwrap::surface B = make_surface(b);

	std::ofstream outfile;
	outfile.open ("/home/pcm/a.off");
	outfile << "OFF";
	outfile << a << std::endl;
	outfile.close();

	outfile.open ("/home/pcm/b.off");
	outfile << "OFF";
	outfile << b << std::endl;
	outfile.close();

	tetwrap::surface outer = make_outer_surface(xmin, ymin, zmin, xmax, ymax, zmax);

	tetwrap::geometry scene;
	scene.add_component(outer);
	scene.add_component(A);
	scene.add_hole(tetwrap::get_interior_point(A));
	scene.add_component(B);
	scene.add_hole(tetwrap::get_interior_point(B));


	// char in_name[] = "/home/pcm/Dropbox/data/testin";
	// in.save_nodes(in_name);
	// in.save_poly(in_name);

	std::cout << "about to construct input" << std::endl;

	tetgenio in = tetwrap::generate_input(scene);

	std::cout << "constructed input" << std::endl;

	tetgenio out;
	char flags[] = "pq";
	tetrahedralize(flags, &in, &out);

	// char out_name[] = "/home/pcm/Dropbox/data/testout";
	// out.save_nodes(out_name);
	// out.save_elements(out_name);
	// out.save_faces(out_name);

	std::cout << "end of mesh generation" << std::endl;
	return out;
}


void build_mesh(const tetgenio& m, dolfin::Mesh& mesh)
{
  // Clear mesh
  mesh.clear();

  // Create and initialize mesh editor
  dolfin::MeshEditor mesh_editor;
  mesh_editor.open(mesh, 3, 3);
  mesh_editor.init_vertices(m.numberofpoints);
  mesh_editor.init_cells(m.numberoftetrahedra);

  for(int i = 0; i < m.numberofpoints; i++) {
	  dolfin::Point p(m.pointlist[3*i], m.pointlist[3*i+1], m.pointlist[3*i+2]);
	  mesh_editor.add_vertex(i, p);
  }

  for(int i = 0; i < m.numberoftetrahedra; i++) {
	  mesh_editor.add_cell(i, m.tetrahedronlist[4*i], m.tetrahedronlist[4*i+1], m.tetrahedronlist[4*i+2], m.tetrahedronlist[4*i+3]);
  }

  // Close mesh editor
  mesh_editor.close();
}

