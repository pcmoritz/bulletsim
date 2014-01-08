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
	btSphereShape sphere_;
	btCollisionObject sphereObj;
	btDefaultCollisionConfiguration* collisionConfiguration_;
	btCollisionDispatcher* dispatcher_;
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
	: sphere_(0.01) {
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
		collisionConfiguration_ = new btDefaultCollisionConfiguration();
		dispatcher_ = new btCollisionDispatcher(collisionConfiguration_);
		btVector3	worldAabbMin(xmin,ymin,zmin);
		btVector3	worldAabbMax(xmax,ymax,zmax);
		broadphase_ = new btAxisSweep3(worldAabbMin,worldAabbMax);
		world_ = new btCollisionWorld(dispatcher_,broadphase_,collisionConfiguration_);
		sphereObj.setCollisionShape(&sphere_);
		btTransform trans;
		trans.setIdentity();
		shapeObj.setWorldTransform(trans);
		shapeObj.setCollisionShape(shape_);
	}
	bool is_inside(btVector3 p) {
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(p);
		sphereObj.setWorldTransform(trans);
		std::cout << p.x() << " " << p.y() << " " << p.z() << std::endl;
		sphereObj.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
		shapeObj.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
		ContactCallback callback;
		world_->contactPairTest(&shapeObj, &shapeObj, callback);
		return callback.has_collided;
	}

};



