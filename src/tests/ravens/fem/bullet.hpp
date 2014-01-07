// #include <btBvhTriangleMeshShape.h>
#include "TetWrap/tetwrap.hpp"
#include <btBulletDynamicsCommon.h>
// #include <btCollisionWorld.h>

btVector3 from_point(const tetwrap::point& p) {
	return btVector3(p.x(), p.y(), p.z());
}

class ContactCallback : public btCollisionWorld::ContactResultCallback {
	bool* has_collided_;
public:
	ContactCallback(bool* has_collided) : has_collided_(has_collided) {}
	virtual	btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
	{
		*has_collided_ = true;
		return 1.0; // Don't really know what that means
	}
};

class Shape {
	btTriangleMesh* mesh_;
	btBvhTriangleMeshShape* shape_;
	btCollisionWorld* world_;
	btAxisSweep3* broadphase_;
	btSphereShape sphere_;
public:
	~Shape() {
		delete mesh_;
		delete shape_;
		delete world_;
		delete broadphase_;
	}
	Shape(const tetwrap::surface& shape, double xmin, double ymin, double zmin,
			double xmax, double ymax, double zmax)
	: sphere_(0.0) {
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
		shape_ = new btBvhTriangleMeshShape(mesh_, true);
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
		btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
		btVector3	worldAabbMin(xmin,ymin,zmin);
		btVector3	worldAabbMax(xmax,ymax,zmax);
		broadphase_ = new btAxisSweep3(worldAabbMin,worldAabbMax);
		world_ = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);
	}
	bool is_inside(btVector3 p) {
		btCompoundShape compound;
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(p);
		compound.addChildShape(trans, &sphere_);
		bool collision = false;
		ContactCallback callback(&collision);
		world_->contactPairTest((btCollisionObject*)&compound,
				(btCollisionObject*) shape_, callback);
		return collision;
	}

};



