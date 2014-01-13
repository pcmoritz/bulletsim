#include <iostream>
#include <vector>

/*
#include "FEMRegistration.hpp"
#include "SceneGeometry.hpp"
#include "bullet.hpp"

#include "fem/model.h"

#include <dolfin.h>
#include <dolfin/common/MPI.h>
#include <dolfin/log/log.h>

#include <dolfin/mesh/Face.h>
#include <dolfin/mesh/Mesh.h>
#include <dolfin/mesh/MeshEditor.h>
#include <dolfin/mesh/MeshPartitioning.h>

#include <dolfin/plot/plot.h>

#include <btBulletCollisionCommon.h>

*/
#include <assert.h>
#include <cstdlib>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include <random>

typedef std::normal_distribution<double> Gaussian;


int main() {
	int n = 10;

	Gaussian gaussian(0.0, 1.0);
	std::default_random_engine generator;

	std::vector<Eigen::Vector3d> domain_points;
	for(int i = 0; i < n; i++) {
		Eigen::Vector3d p(gaussian(generator),
				gaussian(generator), gaussian(generator));
		domain_points.push_back(p);
	}


	Eigen::Matrix3d M(3, 3);
	M(0, 0) = 1.0;
	M(1, 1) = 1.0/std::sqrt(2);
	M(2, 1) = -1.0/std::sqrt(2);
	M(2, 2) = 1.0/std::sqrt(2);
	M(1, 2) = 1.0/std::sqrt(2);


	Eigen::Vector3d t(3);
	t[0] = 1.5;
	t[1] = 1.2;
	t[2] = 1.1;

	std::vector<Eigen::Vector3d> range_points;
	for(int i = 0; i < n; i++) {
		range_points.push_back(M * domain_points[i] + t);
	}


	auto result = affine_transformation(domain_points, range_points);


	std::cout << result.first << std::endl;
	std::cout << result.second << std::endl;


	/*
	// SceneGeometry cube = load("/home/pcm/Dropbox/data/cube.off");
	// SceneGeometry first = load("/home/pcm/Dropbox/data/standard-geometry-cloth-1.off");
	// SceneGeometry second = load("/home/pcm/Dropbox/data/standard-geometry-cloth-2.off");

	SceneGeometry first = load("/home/pcm/Dropbox/data/1.off");
	SceneGeometry second = load("/home/pcm/Dropbox/data/2.off");
    tetgenio tet = constructMesh(first, second, -15.0, -15.0, 5.0, 15.0, 15.0, 30.0);

    dolfin::Mesh mesh;
    dolfin::MeshFunction<std::size_t> ignore(mesh, 2);
    build_mesh(tet, mesh, ignore);

    dolfin::plot(mesh, "mesh of the new situation");
    dolfin::interactive(true);

    SceneGeometry first_standard = load("/home/pcm/Dropbox/data/standard-geometry-cloth-1.off");
    SceneGeometry second_standard = load("/home/pcm/Dropbox/data/standard-geometry-cloth-2.off");

    // YOUR CODE HERE
    using namespace dolfin;

    double xmin = -15.0;
    double ymin = -15.0;
    double zmin = 5.0;
    double xmax = 15.0;
    double ymax = 15.0;
    double zmax = 30.0;

    Mesh standard_mesh;

    dolfin::MeshFunction<std::size_t> boundary(mesh, 2);
    boundary.init(0);
    boundary.init(1);
    boundary.init(2);
    boundary.init(3);
    tetgenio tet_standard = constructMesh(first_standard, second_standard, xmin, ymin, zmin, xmax, ymax, zmax);
    build_mesh(tet_standard, standard_mesh, boundary);

    std::cout << "second mesh built" << std::endl;

    dolfin::plot(standard_mesh, "mesh of standard situation");
    dolfin::interactive(true);

    dolfin::plot(boundary, "mesh function");
    dolfin::interactive(true);

    // SceneGeometry box = from_tetwrap(make_outer_surface(xmin, ymin, zmin, xmax, ymax, zmax));

    // parameters["allow_extrapolation"] = true;

    model::FunctionSpace V(standard_mesh);

    ObjectToObject left_o2o(btTransform::getIdentity());
    ObjectToObject outer_o2o(btTransform::getIdentity());
    //ObjectToObject right_o2o(second.get_transform());
    ObjectToObject right_o2o(btTransform::getIdentity());

    // Create Dirichlet boundary conditions
      DirichletBC bci1(V, left_o2o, boundary, 2);
      DirichletBC bci2(V, right_o2o, boundary, 3);
      DirichletBC bco(V, outer_o2o, boundary, 4);
      std::vector<const DirichletBC*> bcs;
      bcs.push_back(&bci1);
      bcs.push_back(&bci2);
      bcs.push_back(&bco);

      // Define source and boundary traction functions
      Constant B(0.0, -0.5, 0.0);
      Constant T(0.1,  0.0, 0.0);

      // Define solution function
      Function* u = new Function(V); // TODO Remove memory leak

      // Set material parameters
      const double E  = 10.0;
      const double nu = 0.3;
      Constant mu(E/(2*(1 + nu)));
      Constant lambda(E*nu/((1 + nu)*(1 - 2*nu)));

      // Create (linear) form defining (nonlinear) variational problem
      model::ResidualForm F(V);
      F.mu = mu; F.lmbda = lambda; F.B = B; F.T = T;
      F.u = *u;

      // Create jacobian dF = F' (for use in nonlinear solver).
      model::JacobianForm J(V, V);
      J.mu = mu; J.lmbda = lambda;
      J.u = *u;

      std::cout << "start solving the system" << std::endl;

      // Solve nonlinear variational problem F(u; v) = 0
      solve(F == 0, *u, bcs, J);

      std::cout << (*u)[0](10, 13, 10) << " "
    		  << (*u)[1](10, 13, 10) << " "
    		  << (*u)[2](10, 13, 10) << std::endl;

      dolfin::plot(*u, "function");
      dolfin::interactive(true);

   */
      /*

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

	tetwrap::point p = tetwrap::get_interior_point(A);

	std::cout << p << std::endl;

	btVector3 P(p.x(), p.y(), p.z());
	Shape s(A, xmin, ymin, zmin, xmax, ymax, zmax);
	std::cout << s.is_inside(P) << std::endl;
	*/

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
