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
	std::vector<btVector3> vertices;
	std::vector<Face> faces;
	friend std::ostream& operator<<(std::ostream& out, const SceneGeometry& g);
public:
	void add_vertex(btVector3 v) {
		vertices.push_back(v);
	}
	void add_face(std::size_t v1, std::size_t v2, std::size_t v3, std::size_t v4) {
		faces.push_back(Face(v1, v2, v3, v4));
	}
	std::vector<btVector3> get_vertices() const {
		return vertices;
	}
	std::vector<Face> get_faces() const {
		return faces;
	}
	void append(const SceneGeometry& geometry);

	bool on_boundary(btVector3 p, const SceneGeometry& g);

};

SceneGeometry load(const std::string& file_name);

#endif
