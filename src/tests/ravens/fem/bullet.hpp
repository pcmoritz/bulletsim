// #include <btBvhTriangleMeshShape.h>
#include "TetWrap/tetwrap.hpp"
#include <btBulletDynamicsCommon.h>

btVector3 from_point(const tetwrap::point& p) {
	return btVector3(p.x(), p.y(), p.z());
}

struct ContactCallback : public btCollisionWorld::ContactResultCallback {
	bool has_collided;
	ContactCallback() : has_collided(false) {}
	virtual	btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
	{
		std::cout << "objects collide" << std::endl;
		has_collided = true;
		return 1.0; // Don't really know what that means
	}
};

class Shape {
	btTriangleMesh* mesh_;
	btBvhTriangleMeshShape* shape_;
	btCollisionObject shapeObj;
	btCollisionWorld* world_;
	btAxisSweep3* broadphase_;
	btDefaultCollisionConfiguration* collisionConfiguration_;
	btCollisionDispatcher* dispatcher_;
	double xmax_, ymax_, zmax_;
public:
	~Shape() {
		delete mesh_;
		delete shape_;
		delete world_;
		delete broadphase_;
		delete collisionConfiguration_;
		delete dispatcher_;
	}
	Shape(const tetwrap::surface& shape, double xmin, double ymin, double zmin,
			double xmax, double ymax, double zmax)
	: xmax_(xmax), ymax_(ymax), zmax_(zmax) {
		tetwrap::geometry body = tetwrap::make_geometry(shape);
		tetgenio in = tetwrap::generate_input(body);
		tetgenio out;
		char flags[] = "pQ"; // only do triangulation and be quiet
		tetrahedralize(flags, &in, &out);
		mesh_ = new btTriangleMesh();
		for(int i = 0; i < out.numberoftrifaces; i++) {
			btVector3 corners[4];
			for(int j = 0; j < 4; j++) {
				corners[j] = btVector3(
						out.pointlist[out.trifacelist[3 * i]],
						out.pointlist[out.trifacelist[3 * i + 1]],
						out.pointlist[out.trifacelist[3 * i + 2]]);
			}
			mesh_->addTriangle(corners[0], corners[1], corners[2], corners[3]);
		}
		std::cout << mesh_->getNumTriangles() << std::endl;
		shape_ = new btBvhTriangleMeshShape(mesh_, true);
		shapeObj.setCollisionShape(shape_);
		collisionConfiguration_ = new btDefaultCollisionConfiguration();
		dispatcher_ = new btCollisionDispatcher(collisionConfiguration_);
		btVector3	worldAabbMin(xmin,ymin,zmin);
		btVector3	worldAabbMax(xmax,ymax,zmax);
		broadphase_ = new btAxisSweep3(worldAabbMin,worldAabbMax);
		world_ = new btCollisionWorld(dispatcher_,broadphase_,collisionConfiguration_);
		world_->addCollisionObject(&shapeObj);
	}
	bool is_inside(btVector3 p) {
		btSphereShape* sphere = new btSphereShape(1.0);
		btCollisionObject object;
		object.setCollisionShape(sphere);
		btTransform trans;
	    btMatrix3x3 basis;
	    basis.setIdentity();
		trans.setBasis(basis);
		trans.setOrigin(p);
		object.setWorldTransform(trans);
		// world_->addCollisionObject(&object);
		btVector3 to(xmax_, ymax_, zmax_);
		btCollisionWorld::ClosestRayResultCallback callback(to, p);
		world_->rayTest(to, p, callback);
		std::cout << "test" << std::endl;
		// Careful: we assume that only one object is present in the scene.
		if(callback.hasHit()) {
			return true;
		}
		return false;
	}

};



