#include "SceneGeometry.hpp"
#include <fstream>

std::ostream& operator<<(std::ostream& out, const SceneGeometry& g) {
	// output header
	out << " " << g.vertices.size()
	    << " " << g.faces.size()
	    << " " << "0" << std::endl;
	// output vertices
	for(int i = 0; i < g.vertices.size(); i++) {
		out << g.vertices[i].getX() << " "
		    << g.vertices[i].getY() << " "
		    << g.vertices[i].getZ() << std::endl;
	}
	// output faces
	for(int i = 0; i < g.faces.size(); i++) {
		out << "4 "
		    << g.faces[i].edges[0] << " "
		    << g.faces[i].edges[1] << " "
		    << g.faces[i].edges[2] << " "
		    << g.faces[i].edges[3] << std::endl;
	}
	return out;
}

void SceneGeometry::append(const SceneGeometry& geometry)
{
	std::vector<btVector3> vs = geometry.get_vertices();
	vertices.insert(vertices.end(), vs.begin(), vs.end());
	std::size_t V = vs.size();
	std::vector<Face> fs = geometry.get_faces();
	for(std::size_t i = 0; i < fs.size(); i++) {
		add_face(fs[i].edges[0] + V, fs[i].edges[1] + V, fs[i].edges[2] + V, fs[i].edges[3] + V);
	}
}

void SceneGeometry::load(const std::string& file_name) {
	std::ifstream file;
	file.open(file_name.c_str());
	if(!file.is_open()) {
		std::cout << "Could not load file " << file_name << std::endl;
		std::exit(1);
	}
	std::string header;
	file >> header;
	if(header != "OFF") {
		std::cout << "File is not an OFF file" << std::endl;
		std::exit(1);
	}
	int numVert = 0;
	int numFaces = 0;
	int numEdges = 0;
	std::string token;
	while(file >> token) {

	}
	file.close();
}
