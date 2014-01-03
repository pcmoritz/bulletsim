#include "mesh.hpp"

void build_mesh(const dolfin::csg::C3t3& c3t3, dolfin::Mesh& mesh)
{
  typedef dolfin::csg::C3t3 C3T3;
  typedef C3T3::Triangulation Triangulation;
  typedef Triangulation::Vertex_handle Vertex_handle;

  // CGAL triangulation
  const Triangulation& triangulation = c3t3.triangulation();

  // Clear mesh
  mesh.clear();

  // Count cells in complex
  std::size_t num_cells = 0;
  for(dolfin::csg::C3t3::Cells_in_complex_iterator cit = c3t3.cells_in_complex_begin();
      cit != c3t3.cells_in_complex_end();
      ++cit)
  {
    num_cells++;
  }

  // Create and initialize mesh editor
  dolfin::MeshEditor mesh_editor;
  mesh_editor.open(mesh, 3, 3);
  mesh_editor.init_vertices(triangulation.number_of_vertices());
  mesh_editor.init_cells(num_cells);

  // Add vertices to mesh
  std::size_t vertex_index = 0;
  std::map<Vertex_handle, std::size_t> vertex_id_map;

  for (Triangulation::Finite_vertices_iterator
         cgal_vertex = triangulation.finite_vertices_begin();
       cgal_vertex != triangulation.finite_vertices_end(); ++cgal_vertex)
  {
    vertex_id_map[cgal_vertex] = vertex_index;

      // Get vertex coordinates and add vertex to the mesh
    dolfin::Point p(cgal_vertex->point()[0], cgal_vertex->point()[1], cgal_vertex->point()[2]);
    mesh_editor.add_vertex(vertex_index, p);

    ++vertex_index;
  }

  // Add cells to mesh
  std::size_t cell_index = 0;
  for(dolfin::csg::C3t3::Cells_in_complex_iterator cit = c3t3.cells_in_complex_begin();
      cit != c3t3.cells_in_complex_end();
      ++cit)
  {
    mesh_editor.add_cell(cell_index,
                         vertex_id_map[cit->vertex(0)],
                         vertex_id_map[cit->vertex(1)],
                         vertex_id_map[cit->vertex(2)],
                         vertex_id_map[cit->vertex(3)]);

    ++cell_index;
  }

  // Close mesh editor
  mesh_editor.close();
}

void generate(dolfin::Mesh& mesh, const dolfin::csg::Polyhedron_3& p, double cell_size) {
  dolfin_assert(p.is_pure_triangle());
  dolfin::csg::Mesh_domain domain(p);
  domain.detect_features();
  dolfin::csg::Mesh_criteria criteria(CGAL::parameters::facet_angle = 25,
			      CGAL::parameters::facet_size = cell_size,
			      CGAL::parameters::cell_radius_edge_ratio = 3.0,
			      CGAL::parameters::edge_size = cell_size);

  std::cout << "Generating mesh" << std::endl;
  dolfin::csg::C3t3 c3t3 = CGAL::make_mesh_3<dolfin::csg::C3t3>(domain, criteria,
                                                CGAL::parameters::no_perturb(),
                                                CGAL::parameters::no_exude());
  // optimize mesh
  // std::cout << "Optimizing mesh by odt optimization" << std::endl;
  // odt_optimize_mesh_3(c3t3, domain);
  // std::cout << "Optimizing mesh by lloyd optimization" << std::endl;
  // lloyd_optimize_mesh_3(c3t3, domain);
  // This is too slow. Is it really needed?
  // std::cout << "Optimizing mesh by perturbation" << std::endl;
  // CGAL::perturb_mesh_3(c3t3, domain);
  // std::cout << "Optimizing mesh by sliver exudation" << std::endl;
  // exude_mesh_3(c3t3);

  //std::fstream out;
  //out.open("~/mesh.txt", std::ios::out);
  //out << c3t3;
  //out.close();

  build_mesh(c3t3, mesh);
}

// Test if Point p is on the boundary of the cube which is the
// standard (-1, -1, -1) -- (1, 1, 1) cube transformed by the
// transformation t, within a precision of eps
bool is_on_boundary(const dolfin::csg::Exact_Point_3& p, const Aff_trans_3& t, double eps)
{
  dolfin::csg::Exact_Point_3 q = t.inverse()(p);
  // Test if point is inside of a cube that is a bit larger
  if(!((CGAL::abs(q.x()) <= 1 + eps) &&
       (CGAL::abs(q.y()) <= 1 + eps) &&
       (CGAL::abs(q.z()) <= 1 + eps)))
    return false;
  // Test if point is not inside of a cube that is a bit smaller
  if((CGAL::abs(q.x()) <= 1 - eps) &&
     (CGAL::abs(q.y()) <= 1 - eps) &&
     (CGAL::abs(q.z()) <= 1 - eps))
    return false;
  return true;
}

bool is_inside(const Aff_trans_3& t, double x, double y, double z, double eps)
{
  dolfin::csg::Exact_Point_3 p(x, y, z);
  dolfin::csg::Exact_Point_3 q = t.inverse()(p);
  // Test if point is inside of a cube that is a bit larger
  if(!((CGAL::abs(q.x()) <= 1 + eps) &&
       (CGAL::abs(q.y()) <= 1 + eps) &&
       (CGAL::abs(q.z()) <= 1 + eps)))
    return false;
  return true;
}

