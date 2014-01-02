#ifndef MESH_HPP
#define MESH_HPP

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <fstream>

#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Mesh_polyhedron_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/Cartesian_converter.h>

#include <dolfin/common/MPI.h>
#include <dolfin/log/log.h>
#include <dolfin/mesh/Mesh.h>
#include <dolfin/mesh/MeshEditor.h>
#include <dolfin/mesh/MeshPartitioning.h>

#include <dolfin/plot/plot.h>

#define HAS_CGAL

#include <dolfin/generation/CGALMeshBuilder.h>

#include "cgal_triangulate_polyhedron.h"
#include "cgal_copy_polygon_to.h"
#include "cgal_csg3d.h"

#include <dolfin.h>
#include "model.h"

typedef CGAL::Simple_cartesian<double> IK;
typedef CGAL::Cartesian_converter<IK,dolfin::csg::Exact_Kernel> IK_to_EK;

void generate(dolfin::Mesh& mesh, const dolfin::csg::Polyhedron_3& p, double cell_size);

// The transformation first scales the object by the vector (a, b, c),
// then rotates it by the quaternion (x, y, z, w) and then shifts it
// by the vector (t, u, v).
template<class K> CGAL::Aff_transformation_3<K>
cgal_transformation(double a, double b, double c,
		    double x, double y, double z, double w,
		    double t, double u, double v);



#endif
