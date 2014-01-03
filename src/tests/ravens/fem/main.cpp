#include "FEMRegistration.hpp"
#include "SceneGeometry.hpp"

int main() {
	SceneGeometry cube = load("/home/pcm/data/cube.off");
	SceneGeometry first = load("/home/pcm/data/standard-geometry-cloth-1.off");
	SceneGeometry second = load("/home/pcm/data/standard-geometry-cloth-2.off");
}
