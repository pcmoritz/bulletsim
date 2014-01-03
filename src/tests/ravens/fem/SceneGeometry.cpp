#include "SceneGeometry.hpp"
#include <fstream>
#include <iostream>
#include <cstdlib>

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

SceneGeometry load(const std::string& file_name) {
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

	SceneGeometry g;

	std::string token;

	file >> token;
	int numVert = std::atoi(token.c_str());
	file >> token;
	int numFaces = std::atoi(token.c_str());
	file >> token;
	int numEdges = std::atoi(token.c_str());

	std::string coord0_tok, coord1_tok, coord2_tok;
	double coord0, coord1, coord2;
	for(int i = 1; i < numVert; i++) {
	  file >> coord0_tok;
	  file >> coord1_tok;
	  file >> coord2_tok;
	  coord0 = std::atof(coord0_tok.c_str());
	  coord1 = std::atof(coord1_tok.c_str());
	  coord2 = std::atof(coord2_tok.c_str());
	  
	  g.add_vertex(btVector3(coord0, coord1, coord2));
	}

	std::string ind0_tok, ind1_tok, ind2_tok, ind3_tok, n_vertices;
	size_t ind0, ind1, ind2, ind3;
	for(int i = 1; i < numFaces; i++) {
	  file >> n_vertices;
	  file >> ind0_tok;
	  file >> ind1_tok;
	  file >> ind2_tok;
	  file >> ind3_tok;
	  ind0 = std::atoi(ind0_tok.c_str());
	  ind1 = std::atoi(ind1_tok.c_str());
	  ind2 = std::atoi(ind2_tok.c_str());
	  ind3 = std::atoi(ind3_tok.c_str());

	  g.add_face(ind0, ind1, ind2, ind3);
	}

	file.close();

	return g;
}

bool on_boundary(btVector3 p, const SceneGeometry& g)
{
  double tol = 0.000001;

  std::vector<btVector3> vertices = g.get_vertices();
  std::vector<Face> faces = g.get_faces();

  for(int i = 0; i < faces.size(); i++) {
    btVector3 c0 = vertices[faces[i].corners[0]];
    btVector3 c1 = vertices[faces[i].corners[1]];
    btVector3 c2 = vertices[faces[i].corners[2]];

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

  for(int i = 0; i < faces.size(); i++) {
    btVector3 c2 = vertices[faces[i].corners[2]];
    btVector3 c3 = vertices[faces[i].corners[3]];
    btVector3 c0 = vertices[faces[i].corners[0]];

    // test if p and c2, c3, c0 lie in same plane
    btVector3 perp = ((c3 - c2).cross(c0 - c2)).normalize();
    btScalar dist_from_plane = perp.dot(p - c2);
    if (dist_from_plane > tol || dist_from_plane < -tol)
      continue;

    // test if p and c2 lie on the same side of the line c3c0
    if (((p - c3).cross(c0 - c3)).dot((c2 - c3).cross(c0 - c3)) < 0)
      continue;
    // test if p and c3 lie on the same side of the line c2c0
    if (((p - c2).cross(c0 - c2)).dot((c3 - c2).cross(c0 - c2)) < 0)
      continue;
    // test if p and c0 lie on the same side of the line c2c3
    if (((p - c2).cross(c3 - c2)).dot((c0 - c2).cross(c3 - c2)) < 0)
      continue;

    return true;
  }

  return false;
}
