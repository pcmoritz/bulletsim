#include "userconfig.h"

#include <vector>
#include <string>
#include <iostream>
using std::vector;
using std::string;

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

// custom parsers for vector datatypes
static void validate(boost::any &v, const vector<string> &vals, btVector3 *, int) {
    if (vals.size() != 3)
        throw po::validation_error(po::validation_error::invalid_option_value,
            "Invalid vector specification");
    v = btVector3(lexical_cast<btScalar>(vals[0]),
                  lexical_cast<btScalar>(vals[1]),
                  lexical_cast<btScalar>(vals[2]));
}

#if 0
static void validate(boost::any &v, const vector<string> &vals, osg::Vec3 *, int) {
    if (vals.size() != 3)
        throw po::validation_error(po::validation_error::invalid_option_value,
            "Invalid vector specification");
    v = osg::Vec3(lexical_cast<osg::Vec3::value_type>(vals[0]),
                  lexical_cast<osg::Vec3::value_type>(vals[1]),
                  lexical_cast<osg::Vec3::value_type>(vals[2]));
}
#endif

// static ConfigData* ConfigData::Inst() {
//   if (instancePtr == NULL) instancePtr = new ConfigData();
//   return instancePtr;
// }

ConfigData::ConfigData() {
    opts.add_options()
    ("help", "produce help message")
        OPT(verbose, bool, false, "verbose")
        OPT_MULTI(bullet.gravity, btVector3, btVector3(0., 0., -9.8), "gravity")
        OPT(bullet.maxSubSteps, int, 200, "maximum Bullet internal substeps per simulation step")
        OPT(bullet.internalTimeStep, float, 1./200., "internal Bullet timestep")
        OPT(bullet.friction, float, 1., "default friction coefficient for rigid bodies")
        OPT(bullet.restitution, float, 1., "default restitution coefficient for rigid bodies")

        OPT(scene.enableIK, bool, true, "enable OpenRAVE IK for the PR2")
        OPT(scene.enableHaptics, bool, false, "enable haptics for the PR2")
        OPT(scene.enableRobot, bool, true, "enable the PR2")
        OPT(scene.scale, btScalar, 1.0, "scaling factor (1 = 1 meter)")

        OPT_MULTI(viewer.cameraHomePosition, btVector3, btVector3(5, 0, 5), "camera position on startup")
        OPT(viewer.windowWidth, int, 800, "viewer window width")
        OPT(viewer.windowHeight, int, 800, "viewer window height");
}

void ConfigData::read(int argc, char *argv[]) {
    po::variables_map vm;
 po::store(po::parse_command_line(argc, argv, opts), vm);

  po::notify(vm);
}

ConfigData* configData = new ConfigData;
ConfigData* getConfigData() {return configData;}
void setConfigData(ConfigData* newConfigData) {configData = newConfigData;}