#ifndef FEM_REGISTRATION_HPP
#define FEM_REGISTRATION_HPP

#include "SceneGeometry.hpp"
#include <vector>
#include <btBulletDynamicsCommon.h>

#include <dolfin/mesh/Mesh.h>

#define TETLIBRARY
#include "TetGen/tetgen.h"


struct NonRigidTransform {
	std::vector<btVector3> operator()(const std::vector<btVector3>& v);
};

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

	NonRigidTransform constructTransform();
};

tetgenio constructMesh(const SceneGeometry& cloth1, const SceneGeometry& cloth2,
		double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);

void build_mesh(const tetgenio& m, dolfin::Mesh& mesh);

#endif
