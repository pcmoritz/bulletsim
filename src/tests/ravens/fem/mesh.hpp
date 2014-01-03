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

typedef CGAL::Aff_transformation_3<dolfin::csg::Exact_Kernel> Aff_trans_3;

void generate(dolfin::Mesh& mesh, const dolfin::csg::Polyhedron_3& p, double cell_size);

// The transformation first scales the object by the vector (a, b, c),
// then rotates it by the quaternion (x, y, z, w) and then shifts it
// by the vector (t, u, v).
template<class K>
CGAL::Aff_transformation_3<K>
cgal_transformation(double a, double b, double c,
		    double x, double y, double z, double w,
		    double t, double u, double v)
{
  double S = std::sin(w);
  double C = std::cos(w);
  // Scaling
  CGAL::Aff_transformation_3<K> scal(a, 0, 0, 0, b, 0, 0, 0, c);

  // Rotation
  CGAL::Aff_transformation_3<K>
    rot(C + x*x*(1-C), x*y*(1-C) - z*S, x*z*(1-C) + y*S,
      y*x*(1-C) + z*S, C + y*y*(1-C), y*z*(1-C) - x*S,
      z*x*(1-C) - y*S, z*y*(1-C) + x*S, C + z*z*(1-C));

  // Translation
  CGAL::Aff_transformation_3<dolfin::csg::Exact_Kernel>
    trans(1, 0, 0, t, 0, 1, 0, u, 0, 0, 1, v);

  return trans * rot * scal;
}

bool is_inside(const Aff_trans_3& t, double x, double y, double z, double eps = 1e-7);
bool is_on_boundary(const dolfin::csg::Exact_Point_3& p, const Aff_trans_3& t, double eps = 1e-7);

struct CubeDomain : public dolfin::SubDomain
{
  Aff_trans_3 trans;
  std::string debug;
  CubeDomain(const Aff_trans_3& t, std::string d)
    : trans(t), debug(d) {}
  bool inside(const dolfin::Array<double>& x, bool on_boundary) const
  {
    // std::cout << x[0] << " " << x[1] << " " << x[2];
    bool b = is_on_boundary(dolfin::csg::Exact_Point_3(x[0], x[1], x[2]), trans);
    // std::cout << debug << " " << b << on_boundary << std::endl;
    return b && on_boundary;
  }
};

// Evaluate the transformation from the boundary of one cube to the
// boundary of a transformed cube.
struct CubeToCube : public dolfin::Expression
{
  Aff_trans_3 first_cube;
  Aff_trans_3 second_cube;
  CubeToCube (const Aff_trans_3& a, const Aff_trans_3& b)
    : dolfin::Expression(3), first_cube(a), second_cube(b) {}
  void eval(dolfin::Array<double>& values, const dolfin::Array<double>& x) const
  {
    IK_to_EK to_exact;
    dolfin::csg::Exact_Point_3 p(to_exact(x[0]), to_exact(x[1]), to_exact(x[2]));
    (second_cube * first_cube.inverse())(p);
    values[0] = CGAL::to_double(p[0]);
    values[1] = CGAL::to_double(p[1]);
    values[2] = CGAL::to_double(p[2]);
  }
};



#endif
