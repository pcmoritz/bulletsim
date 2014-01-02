#ifndef FEM_REGISTRATION_HPP
#define FEM_REGISTRATION_HPP

#include "SceneGeometry.hpp"
#include <vector>
#include <btBulletDynamicsCommon.h>

struct Transform {
	std::vector<btVector3> operator()(const std::vector<btVector3>& v);
};

class FEMRegistration
{
private:
	SceneGeometry geometry;
	btScalar xmin, ymin, xmax, ymax;
public:
	// Construct registration module out of geometry and bounding box around
	// the geometry. The bounding box is fixed by the corners a and b.
	FEMRegistration(const SceneGeometry& geometry, btVector3 a, btVector3 b) {
		this->geometry = geometry;
		xmin = a.getX();
		ymin = a.getY();
		xmax = a.getX();
		ymax = a.getY();
	}
	Transform constructTransform();
};

#endif
