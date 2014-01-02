#ifndef FEM_REGISTRATION_HPP
#define FEM_REGISTRATION_HPP

#include "SceneGeometry.hpp"
#include <vector>
#include <btBulletDynamicsCommon.h>
#include "mesh.hpp"

struct Transform {
	std::vector<btVector3> operator()(const std::vector<btVector3>& v);
};

struct Meshes {
	dolfin::Mesh domain_mesh;
	dolfin::Mesh range_mesh;
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
		xmax = a.getX();
		ymax = a.getY();
		zmax = a.getZ();
	}
	Meshes constructMesh(const SceneGeometry& cloth1, const SceneGeometry& cloth2);
	Transform constructTransform();
};

#endif
