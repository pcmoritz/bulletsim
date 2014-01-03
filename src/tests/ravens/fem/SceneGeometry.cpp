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
		    << g.faces[i].corners[0] << " "
		    << g.faces[i].corners[1] << " "
		    << g.faces[i].corners[2] << " "
		    << g.faces[i].corners[3] << std::endl;
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
		add_face(fs[i].corners[0] + V, fs[i].corners[1] + V, fs[i].corners[2] + V, fs[i].corners[3] + V);
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

bool SceneGeometry::on_boundary(btVector3 p, const SceneGeometry& g)
{
  double tol = 0.000001;

  for(int i = 0; i < g.faces.size(); i++) {
    btVector3 c0 = vertices[g.faces[i].corners[0]];
    btVector3 c1 = vertices[g.faces[i].corners[1]];
    btVector3 c2 = vertices[g.faces[i].corners[2]];

    // test if p and c0, c1, c2 lie in same plane
    btVector3 perp = ((c1 - c0).cross(c2 - c0)).normalize();
    btScalar dist_from_plane = perp.dot(p - c0);
    if (dist_from_plane > tol || dist_from_plane < -tol)
      continue;
    
    // test if p and c0 lie on the same side of the line c1c2
    if (((p - c1).cross(c2 - c1)).dot((c0 - c1).cross(c2 - c1)) < 0)
      continue;
    // test if p and c1 lie on the same side of the line c0c2
    if (((p - c0).cross(c2 - c0)).dot((c1 - c0).cross(c2 - c0)) < 0)
      continue;
    // test if p and c2 lie on the same side of the line c0c1
    if (((p - c0).cross(c1 - c0)).dot((c2 - c0).cross(c1 - c0)) < 0)
      continue;

    return true;
  }

  for(int i = 0; i < g.faces.size(); i++) {
    btVector3 c1 = vertices[g.faces[i].corners[1]];
    btVector3 c2 = vertices[g.faces[i].corners[2]];
    btVector3 c3 = vertices[g.faces[i].corners[3]];

    // test if p and c1, c2, c3 lie in same plane
    btVector3 perp = ((c2 - c1).cross(c3 - c1)).normalize();
    btScalar dist_from_plane = perp.dot(p - c1);
    if (dist_from_plane > tol || dist_from_plane < -tol)
      continue;

    // test if p and c1 lie on the same side of the line c2c3
    if (((p - c2).cross(c3 - c2)).dot((c1 - c2).cross(c3 - c2)) < 0)
      continue;
    // test if p and c2 lie on the same side of the line c1c3
    if (((p - c1).cross(c3 - c1)).dot((c2 - c1).cross(c3 - c1)) < 0)
      continue;
    // test if p and c3 lie on the same side of the line c1c2
    if (((p - c1).cross(c2 - c1)).dot((c3 - c1).cross(c2 - c1)) < 0)
      continue;

    return true;
  }

  return false;
}
