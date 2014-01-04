#include "FEMRegistration.hpp"
#include <dolfin/mesh/MeshEditor.h>

NonRigidTransform FEMRegistration::constructTransform()
{
	return NonRigidTransform();
}

void setfacet(tetgenio::facet *f, int a, int b, int c, int d)
{
	f->numberofpolygons = 1;
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;
	tetgenio::polygon *p = &f->polygonlist[0];
	p->numberofvertices = 4;
	p->vertexlist = new int[p->numberofvertices];
	p->vertexlist[0] = a;
	p->vertexlist[1] = b;
	p->vertexlist[2] = c;
	p->vertexlist[3] = d;
}

tetgenio constructMesh(const SceneGeometry& a, const SceneGeometry& b,
		double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
{
	std::vector<btVector3> a_vert = a.get_vertices();
	std::vector<btVector3> b_vert = b.get_vertices();
	std::cout << "sizes: " << a_vert.size() << " " << b_vert.size() << std::endl;
	tetgenio in;
	// All indices start from 0
	in.firstnumber = 0;
	// The 4 is for the vertices of the outer boundary
	in.numberofpoints = a_vert.size() + b_vert.size() + 8;
	in.pointlist = new REAL[in.numberofpoints * 3];
	for(int i = 0; i < a_vert.size(); i++) {
		in.pointlist[3 * i] = a_vert[i].getX();
		in.pointlist[3 * i + 1] = a_vert[i].getY();
		in.pointlist[3 * i + 2] = a_vert[i].getZ();
	}

	int V = 3 * a_vert.size();
	for(int i = 0; i < b_vert.size(); i++) {
		in.pointlist[V + 3 * i] = b_vert[i].getX();
		in.pointlist[V + 3 * i + 1] = b_vert[i].getY();
		in.pointlist[V + 3 * i + 2] = b_vert[i].getZ();
	}

	// The last few vertices are the ones of the outer boundary
	V = 3 * a_vert.size() + 3 * b_vert.size();

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
	in.pointlist[V + 12] = xmin;
	in.pointlist[V + 13] = ymin;
	in.pointlist[V + 14] = zmax;
	// Sixth point:
	in.pointlist[V + 15] = xmax;
	in.pointlist[V + 16] = ymin;
	in.pointlist[V + 17] = zmax;
	// Seventh point:
	in.pointlist[V + 18] = xmax;
	in.pointlist[V + 19] = ymax;
	in.pointlist[V + 20] = zmax;
	// Eight point:
	in.pointlist[V + 21] = xmin;
	in.pointlist[V + 22] = ymax;
	in.pointlist[V + 23] = zmax;

	// Set the facets:
	std::vector<Face> a_faces = a.get_faces();
	std::vector<Face> b_faces = b.get_faces();
	// The 6 is for the faces on the outside boundary
	in.numberoffacets = a_faces.size() + b_faces.size() + 6;


	in.facetlist = new tetgenio::facet[in.numberoffacets];
	in.facetmarkerlist = new int[in.numberoffacets];
	// Facets for the first object:
	for(int i = 0; i < a_faces.size(); i++) {
		tetgenio::facet *f = &in.facetlist[i];
		setfacet(f, a_faces[i].corners[0],
					a_faces[i].corners[1],
					a_faces[i].corners[2],
					a_faces[i].corners[3]);
	}

	// Facets for the second object:
	int F = a_faces.size();
	for(int i = 0; i < b_faces.size(); i++) {
		tetgenio::facet *f = &in.facetlist[F + i];
		setfacet(f, b_faces[i].corners[0] + a_vert.size(),
					b_faces[i].corners[1] + a_vert.size(),
					b_faces[i].corners[2] + a_vert.size(),
					b_faces[i].corners[3] + a_vert.size());
	}

	int W = a_vert.size() + b_vert.size();

	// Facets for the boundary:
	F = a_faces.size() + b_faces.size();
	// bottom
	setfacet(&in.facetlist[F], W, W+1, W+2, W+3);
	// top
	setfacet(&in.facetlist[F + 1], W+4, W+5, W+6, W+7);
	// left
	setfacet(&in.facetlist[F + 2], W, W+4, W+7, W+3);
	// front
	setfacet(&in.facetlist[F + 3], W, W+1, W+5, W+4);
	// right
	setfacet(&in.facetlist[F + 4], W+1, W+5, W+6, W+2);
	// back
	setfacet(&in.facetlist[F + 5], W+2, W+6, W+7, W+3);

	in.numberofholes = 2;
	in.holelist = new REAL[in.numberofholes * 3];
	in.holelist[0] = a.get_center().getX();
	in.holelist[1] = a.get_center().getY();
	in.holelist[2] = a.get_center().getZ();
	in.holelist[3] = b.get_center().getX();
	in.holelist[4] = b.get_center().getY();
	in.holelist[5] = b.get_center().getZ();

	// char in_name[] = "/home/pcm/Dropbox/data/testin";
	// in.save_nodes(in_name);
	// in.save_poly(in_name);

	tetgenio out;
	char flags[] = "pq1.414a0.1";
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

