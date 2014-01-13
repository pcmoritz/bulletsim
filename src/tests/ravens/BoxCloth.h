#pragma once
#include <simulation/environment.h>
#include <simulation/basicobjects.h>
#include <simulation/openravesupport.h>
#include <simulation/simplescene.h>

#include "btBulletDynamicsCommon.h"
#include <vector>
#include <utils/config.h>
#include <utility>

#include "fem/SceneGeometry.hpp"

class CustomScene;

class BoxCloth : public CompoundObject<BulletObject> {

	Scene & scene;

public:

	// number of boxes along the x-dimension
	int n;

	// number of boxes along the y-dimension
	int m;

	// size of the side of the box objects along x and y dimensions
	btScalar s;

	// height of the box-objects along the z dimension
	btScalar h;

	// center of the cloth
	const btVector3   center;
	const btTransform rot;

	//indices of the squares in the 2d grid where holes should be located.
	vector<unsigned int> hole_is, hole_js;

	// grid(i,j) |--> index of square object in this->children
	std::map<std::pair<unsigned int, unsigned int>, unsigned int> grid_to_obj_inds;

	// spring/ constraints parameters
	float angStiffness;
	float linDamping;
	float angDamping;
	float angLimit;


	bool isHole(unsigned int i, unsigned int j);
	unsigned int getSerializedIndex(unsigned int i, unsigned int j);

	typedef boost::shared_ptr<BoxCloth> Ptr;
	std::vector<boost::shared_ptr<btCollisionShape> > shapes;
	std::vector<RaveObject::Ptr>    holes;
	std::vector<RaveObject::Ptr>    raveBoxes;
	std::vector<BulletConstraint::Ptr> joints;

	BoxCloth(Scene &s, unsigned int n_, unsigned int m_, vector<unsigned int> hole_is_, vector<unsigned int> hole_js_,
			  btScalar s_=METERS*0.02, btScalar h_=METERS*0.001, btVector3 center_=btVector3(0,0,0),
			  btTransform rot=btTransform::getIdentity(),
			  float angStiffness_=1e0, float linDamping_=3, float angDamping_=1, float angLimit_=0.5);


	void makeHole (btTransform &tfm);
	void addChild (float mass, btVector3 hfExtents, btTransform &tfm);
	void addHoleConstraint (vector<btVector3> &offsetA, vector<btVector3> &offsetB,
	  	  	  				const boost::shared_ptr<btRigidBody> rbA,
	  	  	  				const boost::shared_ptr<btRigidBody>& rbB);

	string objectType () {return "BoxCloth";}

	void getBoxClothPoints(vector< pair<int, int> > indices, vector<btVector3> & boxPoints);
	void getBoxClothHoles(vector<btVector3> & holePoints);

	void createBoxTransforms(vector<btTransform> &transforms, unsigned int n, unsigned int m, btScalar s, btVector3 center);


	void init();
	void destroy();

	// Get the cloth geometry in OFF format; if this it to be combined with other parts of the scene, the indices have to be remapped
	SceneGeometry getBoxClothGeometry();
	// Get the eight corners of the cloth for registering the cloth to each other
	std::vector<btVector3> getCorners();
};
