#include "mesh.hpp"
#include "model.h"

using namespace dolfin;

int main() {
  std::string off_file = "../cube.off";
  csg::Exact_Polyhedron_3 cube; // unit cube
  std::cout << "reading file " << off_file << std::endl;
  std::ifstream file(off_file.c_str());
  file >> cube;
  std::cout << "done reading file." << std::endl;
  double cell_size = 1.0;
  bool detect_sharp_features = true;
  Mesh m;
  Mesh m2;

  csg::Exact_Polyhedron_3 outer(cube);

  // scale the outer box
  CGAL::Aff_transformation_3<csg::Exact_Kernel>
    St(4, 0, 0, 0, 0, 3, 0, 0, 0, 0, 2, 0);
  std::transform(outer.points_begin(), outer.points_end(),
  		 outer.points_begin(), St);

  // scale the inner box
  CGAL::Aff_transformation_3<csg::Exact_Kernel>
    Et(1, 0, 0, -1.5, 0, 1, 0, 0, 0, 0, 0.2, 0);
  csg::Nef_polyhedron_3 Omega(outer);
  csg::Nef_polyhedron_3 Omega2(outer);
  csg::Exact_Polyhedron_3 first_inner(cube);
  std::transform(first_inner.points_begin(), first_inner.points_end(),
  		 first_inner.points_begin(), Et);

  csg::Exact_Polyhedron_3 second_inner(cube);
  CGAL::Aff_transformation_3<csg::Exact_Kernel> TT =
    cgal_transformation<csg::Exact_Kernel>(1.0, 1.0, 0.2, 0.0, 0.0, 1.0, 0.3, 1.5, 0.0, 0.0);
  std::transform(second_inner.points_begin(), second_inner.points_end(),
  		 second_inner.points_begin(), TT);

  csg::Exact_Polyhedron_3 third_inner(cube);
  CGAL::Aff_transformation_3<csg::Exact_Kernel> TTT =
    cgal_transformation<csg::Exact_Kernel>(1.0, 1.5, 0.4, 1.0, 0.0, 0.0, 0.5, 2.0, 1.5, 0.5);
  std::transform(third_inner.points_begin(), third_inner.points_end(),
  		 third_inner.points_begin(), TTT);

  Omega -= first_inner;
  Omega -= second_inner;

  Omega2 -= first_inner;
  Omega2 -= third_inner;

  csg::Exact_Polyhedron_3 p;
  Omega.convert_to_polyhedron(p);
  csg::Polyhedron_3 q;
  copy_to(p, q);

  generate(m, q, cell_size);

  plot(m, "mesh of a cube");
  interactive(true);

  csg::Exact_Polyhedron_3 p2;
  Omega2.convert_to_polyhedron(p2);
  csg::Polyhedron_3 q2;
  copy_to(p2, q2);

  generate(m2, q2, cell_size);

  plot(m2, "mesh of a cube");
  interactive(true);

  std::cout << "Solving the variational problem" << std::endl;
  model::FunctionSpace V(m);

  CubeDomain first_inner_domain(Et, "first inner");
  CubeToCube first_map(Et, Et);
  CubeDomain second_inner_domain(TT, "second inner");
  CubeToCube second_map(TT, TTT);
  CubeDomain outer_domain(St, "outer");
  CubeToCube o2o(St, St);

  // Create Dirichlet boundary conditions
  DirichletBC bci1(V, first_map, first_inner_domain);
  DirichletBC bci2(V, second_map, second_inner_domain);
  DirichletBC bco(V, o2o, outer_domain);
  std::vector<const BoundaryCondition*> bcs;
  bcs.push_back(&bci1);
  bcs.push_back(&bci2);
  bcs.push_back(&bco);

  // Define source and boundary traction functions
  Constant B(0.0, -0.5, 0.0);
  Constant T(0.1,  0.0, 0.0);

  // Define solution function
  Function u(V);

  // Set material parameters
  const double E  = 10.0;
  const double nu = 0.3;
  Constant mu(E/(2*(1 + nu)));
  Constant lambda(E*nu/((1 + nu)*(1 - 2*nu)));

  // Create (linear) form defining (nonlinear) variational problem
  model::ResidualForm F(V);
  F.mu = mu; F.lmbda = lambda; F.B = B; F.T = T; F.u = u;

  // Create jacobian dF = F' (for use in nonlinear solver).
  model::JacobianForm J(V, V);
  J.mu = mu; J.lmbda = lambda; J.u = u;

  // Solve nonlinear variational problem F(u; v) = 0
  solve(F == 0, u, bcs, J);

  // Plot solution
  // plot(u);
  // interactive();

  std::getchar();

  int N = 10;
  double wx = 7.8/(N-1);
  double wy = 5.8/(N-1);
  double wz = 3.8/(N-1);

  std::fstream out;
  out.open("/home/robertnishihara/function-values.txt", std::ios::out);

  for(int x = 0; x < N; x++) {
    for(int y = 0; y < N; y++) {
      for(int z = 0; z < N; z++) {
	Array<double> values(3);
	Array<double> location(3);
	location[0] = x * wx - 3.9;
	location[1] = y * wy - 2.9;
	location[2] = z * wz - 1.9;

	if(is_inside(TT, location[0], location[1], location[2]) ||
	   is_inside(Et, location[0], location[1], location[2])) {
	  out << "NA" << std::endl;
	  //printf("inside\n");
	}
	else {
	  u.eval(values, location);
	  out << values[0] << " " << values[1] << " " << values[2] << std::endl;
	  //printf("%f %f %f\n", values[0], values[1], values[2]);
	}
      }
    }
  }

  out.close();
}
