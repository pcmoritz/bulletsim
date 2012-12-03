#include "simulation/hand.h"
#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/bullet_io.h"
#include "utils/config.h"

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 10;

	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.read(argc, argv);

	Scene scene;

	HumanHandObject::Ptr hand(new HumanHandObject(scene.rave));
	scene.env->add(hand);
	cout << "joint angles " << hand->getJointAngles() << endl;

	scene.addVoidKeyCallback('q',boost::bind(exit, 0));

	scene.startViewer();
	scene.startLoop();

	return 0;
}
