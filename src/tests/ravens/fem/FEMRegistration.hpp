#ifndef FEM_REGISTRATION_HPP
#define FEM_REGISTRATION_HPP

#include "SceneGeometry.hpp"
#include <vector>
#include <btBulletDynamicsCommon.h>

#include <dolfin/mesh/Mesh.h>
#include <dolfin/mesh/Face.h>

#include "TetWrap/tetwrap.hpp"
#include <fstream>

#include <Eigen/Core>
#include <Eigen/LU>



// struct NonRigidTransform {
// 	std::vector<btVector3> operator()(const std::vector<btVector3>& v);
// };

struct Meshes {
	dolfin::Mesh* domain_mesh;
	dolfin::Mesh* range_mesh;
	Meshes() {
		domain_mesh = new dolfin::Mesh();
		range_mesh = new dolfin::Mesh();
	}
	~Meshes() {
		std::cout << "deleting the meshes" << std::endl;
		delete domain_mesh;
		delete range_mesh;
	}
};

class FEMRegistration
{
private:
	double xmin, ymin, zmin, xmax, ymax, zmax;
public:
	// Construct registration module with bounding box around
	// the geometry. The bounding box is fixed by the corners a and b.
	FEMRegistration(btVector3 a, btVector3 b) {
		xmin = a.getX();
		ymin = a.getY();
		zmin = a.getZ();
		xmax = b.getX();
		ymax = b.getY();
		zmax = b.getZ();
	}

	// NonRigidTransform constructTransform();
};

// A transformation of the form x => R * x + t
typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> RigidTransform;
typedef std::vector<Eigen::Vector3d> PointCloud;

RigidTransform affine_transformation(const PointCloud& domain_points, const PointCloud& range_points);

btTransform constructTransform(const std::string& domain_file_name, const PointCloud& range_points);

// Evaluate the transformation from the boundary of one cube to the
// boundary of a transformed cube.
struct ObjectToObject : public dolfin::Expression
{
  btTransform transform;
  ObjectToObject (const btTransform& transform) : dolfin::Expression(3)
  {
	  this->transform = transform;
  }
  void eval(dolfin::Array<double>& values, const dolfin::Array<double>& x) const
  {
	btVector3 v(x[0], x[1], x[2]);
	btVector3 w = transform * v;
	// btVector3 w = v;
    values[0] = w[0];
    values[1] = w[1];
    values[2] = w[2];
  }
  void write_to_file(const std::string& filename) {
	  std::ofstream out(filename.c_str());
	  btVector3 origin = transform.getOrigin();
	  out << origin[0] << ", "
		  << origin[1] << ", "
		  << origin[2] << std::endl;

	  btMatrix3x3 matrix = transform.getBasis();
	  for(int i = 0; i < 3; i++) {
			  out << matrix[i][0] << ", "
				  << matrix[i][1] << ", "
				  << matrix[i][2] << std::endl;
	  }
	  out.close();
  }
};

/*

struct ClothDomain : public dolfin::SubDomain
{
  SceneGeometry geometry;
  std::string debug;
  ClothDomain(const SceneGeometry& g, std::string d)
    : geometry(g), debug(d) {}
  bool inside(const dolfin::Array<double>& x, bool boundary_point) const
  {
    std::cout << x[0] << " " << x[1] << " " << x[2];
    bool b = on_boundary(btVector3(x[0], x[1], x[2]), geometry);
    std::cout << debug << " " << b << boundary_point << std::endl;
    return b && boundary_point;
  }
};

*/

tetgenio constructMesh(const SceneGeometry& cloth1, const SceneGeometry& cloth2,
		double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);

void build_mesh(const tetgenio& m, dolfin::Mesh& mesh, dolfin::MeshFunction<std::size_t>& boundary);

tetwrap::facet make_facet(const Face& f);
tetwrap::surface make_surface(int marker, const SceneGeometry& g);
tetwrap::surface make_outer_surface(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);

#endif
