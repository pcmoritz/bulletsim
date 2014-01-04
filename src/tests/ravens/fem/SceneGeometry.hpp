// A geometric description of the scene that is suitable for
// subsequent mesh generation

#ifndef SCENE_GEOMETRY_HPP
#define SCENE_GEOMETRY_HPP

#include <vector>
#include <btBulletDynamicsCommon.h>
#include <ostream>

struct Face {
	std::size_t corners[4];
	Face(std::size_t v1, std::size_t v2, std::size_t v3, std::size_t v4) {
		corners[0] = v1;
		corners[1] = v2;
		corners[2] = v3;
		corners[3] = v4;
	}
};

class SceneGeometry {
	btVector3 center;
	std::vector<btVector3> vertices;
	std::vector<Face> faces;
	btTransform transform; // usually, this is just ignored
	friend std::ostream& operator<<(std::ostream& out, const SceneGeometry& g);
public:
	// SceneGeometry(const btTransform& transform) {
	//	this->transform = transform;
	// }
	SceneGeometry() {
		this->transform = btTransform::getIdentity();
	}
	void add_vertex(btVector3 v) {
		vertices.push_back(v);
	}
	void add_face(std::size_t v1, std::size_t v2, std::size_t v3, std::size_t v4) {
		faces.push_back(Face(v1, v2, v3, v4));
	}
	void set_center(btVector3 center) {
		this->center = center;
	}
	void set_transform(const btTransform& transform) {
		this->transform = transform;
	}
	btTransform get_transform() {
		return transform;
	}
	btVector3 get_center() const {
		return center;
	}
	std::vector<btVector3> get_vertices() const {
		return vertices;
	}
	std::vector<Face> get_faces() const {
		return faces;
	}
	void append(const SceneGeometry& geometry);
};

SceneGeometry load(const std::string& file_name);
bool on_boundary(btVector3 p, const SceneGeometry& g);

#endif
