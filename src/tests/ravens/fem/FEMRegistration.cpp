#include "FEMRegistration.hpp"
#include "SceneGeometry.hpp"
#include <dolfin/mesh/MeshEditor.h>
#include <dolfin/common/Array.h>
#include <dolfin/common/RangedIndexSet.h>
#include <dolfin/mesh/Mesh.h>
#include <dolfin/mesh/MeshData.h>
#include <dolfin/mesh/MeshEntity.h>
#include <dolfin/mesh/MeshEntityIterator.h>
#include <dolfin/mesh/Vertex.h>
#include <dolfin/mesh/MeshFunction.h>
#include <dolfin/mesh/MeshValueCollection.h>
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

tetwrap::surface make_surface(int marker, const SceneGeometry& g) {
	tetwrap::surface result(marker);
	std::vector<btVector3> vert = g.get_vertices();
	std::vector<Face> face = g.get_faces();
	for(int i = 0; i < vert.size(); i++)
		result.add_vertex(tetwrap::point(vert[i].getX(), vert[i].getY(), vert[i].getZ()));
	for(int i = 0; i < face.size(); i++)
		result.add_facet(make_facet(face[i]));
	return result;
}

tetwrap::surface make_outer_surface(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax) {
	tetwrap::surface outer(4);
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

	tetwrap::surface A = make_surface(2, a);
	tetwrap::surface B = make_surface(3, b);

	tetwrap::surface outer = make_outer_surface(xmin, ymin, zmin, xmax, ymax, zmax);

	tetwrap::geometry scene;
	scene.add_component(outer);
	scene.add_component(A);
	scene.add_hole(tetwrap::get_interior_point(A));
	scene.add_component(B);
	scene.add_hole(tetwrap::get_interior_point(B));


	std::cout << "about to construct input" << std::endl;

	tetgenio in = tetwrap::generate_input(scene);

	//char in_name[] = "/home/pcm/Dropbox/data/testin";
	//in.save_nodes(in_name);
	//in.save_poly(in_name);

	std::cout << "constructed input" << std::endl;

	tetgenio out;
	char flags[] = "pqg";
	tetrahedralize(flags, &in, 0);

	char out_name[] = "/home/pcm/Dropbox/data/testout";
	out.save_nodes(out_name);
	out.save_elements(out_name);
	out.save_faces(out_name);

	std::cout << "end of mesh generation" << std::endl;
	return out;
}


// Apply markers for our boundary conditions
void mark_mesh(const tetgenio& m, dolfin::Mesh& mesh, dolfin::MeshFunction<std::size_t>& boundary)
{
	boundary.init(2);
	boundary.set_all(0);
	// Make map of all boundary vertices
	std::map<std::size_t, int> boundary_vertices;
	for(int i = 0; i < m.numberoftrifaces; i++) {
		boundary_vertices[m.trifacelist[3*i]] = m.trifacemarkerlist[i];
		boundary_vertices[m.trifacelist[3*i+1]] = m.trifacemarkerlist[i];
		boundary_vertices[m.trifacelist[3*i+2]] = m.trifacemarkerlist[i];
	}

	// Initialize the mesh such that it can hold facets and facet/cell connections
	mesh.init(2);
	mesh.init(2, 3);

	dolfin::RangedIndexSet boundary_visited(mesh.num_vertices());
	dolfin::RangedIndexSet interior_visited(mesh.num_vertices());
	std::vector<bool> boundary_inside(mesh.num_vertices());
	std::vector<bool> interior_inside(mesh.num_vertices());

	for(dolfin::MeshEntityIterator entity(mesh, 2); !entity.end(); ++entity) {
		// Entity on boundary if entity is a facet?
		bool on_boundary = (entity->num_global_entities(3) == 1);

		// Visited-cache to use for this facet
		dolfin::RangedIndexSet& is_visited = (on_boundary ? boundary_visited : interior_visited);
		std::vector<bool>& is_inside = (on_boundary ? boundary_inside : interior_inside);

		// Start by assuming all points are inside
		bool all_points_inside = true;
		std::size_t boundary_marker = 0;
		if (entity->dim() > 0)
		{
			for(dolfin::VertexIterator vertex(*entity); !vertex.end(); ++vertex) {
				if(is_visited.insert(vertex->index())) {
					is_inside[vertex->index()] = !(boundary_vertices.find(vertex->index()) == boundary_vertices.end());
				}
				if(!is_inside[vertex->index()]) {
					all_points_inside = false;
					break;
				} else {
					boundary_marker = boundary_vertices[vertex->index()];
					std::cout << "marker: " << boundary_marker << std::endl;
				}
			}
			std::cout << "next vertex" << std::endl;
		}
		if(all_points_inside) {
			boundary[entity->index()] = boundary_marker;
		}
	}
}

// Build the mesh and set up the boundary conditions
void build_mesh(const tetgenio& m, dolfin::Mesh& mesh, dolfin::MeshFunction<std::size_t>& boundary)
{
	std::system("gmsh tetgen-tmpfile.1.mesh -3 -o output.msh");
	std::system("python ../convert.py");

	dolfin::File mesh_file("output.xml");
	mesh_file >> mesh;
	dolfin::File boundary_file("output_facet_region.xml");
	boundary_file >> boundary;
}

