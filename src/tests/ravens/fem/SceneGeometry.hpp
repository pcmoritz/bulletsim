// A geometric description of the scene that is suitable for
// subsequent mesh generation

#ifndef SCENE_GENERATION_HPP
#define SCENE_GENERATION_HPP

#include <vector>
#include <btBulletDynamicsCommon.h>
#include <ostream>

struct Face {
	std::size_t edges[4];
	Face(std::size_t e1, std::size_t e2, std::size_t e3, std::size_t e4) {
		edges[0] = e1;
		edges[1] = e2;
		edges[2] = e3;
		edges[3] = e4;
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
	void add_face(std::size_t e1, std::size_t e2, std::size_t e3, std::size_t e4) {
		faces.push_back(Face(e1, e2, e3, e4));
	}
	std::vector<btVector3> get_vertices() const {
		return vertices;
	}
	std::vector<Face> get_faces() const {
		return faces;
	}
	void append(const SceneGeometry& geometry);
};

#endif
