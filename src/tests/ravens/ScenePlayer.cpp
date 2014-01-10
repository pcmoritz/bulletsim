#include "lfd/SegmentDemo.hpp"
#include "ScenePlayer.hpp"
#include "CustomScene.h"

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <algorithm>
#include <utility>
#include "lfd/utils_python.h"
#include <utils/colorize.h>

#include "lfd/TPSBijectWrapper.hpp"

#include "fem/SceneGeometry.hpp"
#include "fem/FEMRegistration.hpp"

#include "fem/model.h"

#include <dolfin.h>
#include <dolfin/common/MPI.h>
#include <dolfin/log/log.h>

#include <dolfin/mesh/Face.h>
#include <dolfin/mesh/Mesh.h>
#include <dolfin/mesh/MeshEditor.h>
#include <dolfin/mesh/MeshPartitioning.h>

#include <dolfin/plot/plot.h>



using namespace std;


pointsToUse all         = {true, true, true};
pointsToUse ropeOnly    = {true, false, false};
pointsToUse boxAndHole  = {false, true, true};
pointsToUse ropeAndHole = {true, false, true};
pointsToUse pointModes[] = {all, ropeOnly, boxAndHole, ropeAndHole};

int ScenePlayer::getCurrentPlayNumber() {
	cout << "GETTING CURRENT PLAY NUM"<<endl;
	ifstream inpfile(runnumfname.c_str());
	unsigned int runnum;

	if(!inpfile.is_open()) {
		cout << "[ERROR : ScenePlayer] : Unable to open run-num file : " << runnumfname << endl;
		exit(-1);
	} else {
		while(!inpfile.eof()) {
			string line;
			getline(inpfile,line);
			vector<string> splitline;
			string buf;
			stringstream ss(line);

			while (ss >> buf)  // extract white-space separated strings on a line
				splitline.push_back(buf);

			if (splitline.size()==0 || splitline[0][0] == '#') { // skip blank lines and comments
				continue;
			} else {
				if(splitline.size() != 1) {
					cout << "[ERROR : ScenePlayer] : Unknown run-num file format."<< endl;
					exit(-1);
				}
				runnum = atoi(splitline[0].c_str());
			}
		}
		inpfile.close();
		cout << "PLAY NUM : "<<runnum<<endl;

		return runnum;
	}
}

vector<int> ScenePlayer::readDemoLibFile() {
	vector<int> demonums;
	ifstream inpfile(demolibfname.c_str());

	if(!inpfile.is_open()) {
		cout << "[ERROR : ScenePlayer] : Unable to open demo library file : " << demolibfname << endl;
		exit(-1);
	} else {
		while(!inpfile.eof()) {
			string line;
			getline(inpfile,line);
			vector<string> splitline;
			string buf;
			stringstream ss(line);

			while (ss >> buf)  // extract white-space separated strings on a line
				splitline.push_back(buf);

			if (splitline.size()==0 || splitline[0][0] == '#') { // skip blank lines and comments
				continue;
			} else {
				if(splitline.size() != 1) {
					cout << "[ERROR : ScenePlayer] : Unknown demo library file format."<< endl;
					exit(-1);
				}
				demonums.push_back(atoi(splitline[0].c_str()));
			}
		}

		inpfile.close();
		return demonums;
	}
}

/** Returns the demo-file number of the the demo which
 *  matches the most to the current setting.
 *
 *  Note: Only the first segment of a demo in the library is used to calculate the cost.
 *
 *  This is done iff findClosestDemo is true. Else getCurrentPlayNumber is used. */
int ScenePlayer::getClosestDemoNum(dolfin::Function* u) {
	// get current scene's points
	vector<btVector3> target_rope, target_box, target_hole;
	scene.sRope->getRopePoints(true, target_rope, 1.0/METERS);
	scene.getBoxPoints(target_box, 1.0/METERS);
	scene.getBoxHoles(target_hole, 1.0/METERS);

	vector<vector<btVector3> > target_clouds;
	target_clouds.push_back(target_rope);
	target_clouds.push_back(target_box);
	target_clouds.push_back(target_hole);

	vector<int> demonums = readDemoLibFile();
	vector<double> warpingDists(demonums.size());

	for(int i=0; i < demonums.size(); i+=1) {
		stringstream demofnamess;
		demofnamess << demodir << "/run" << demonums[i] << ".txt";
		string demofname = demofnamess.str();
		trajSegment::Ptr tseg = Segmenter(demofname).getNextSegment();

		if (tseg) {
			vector<vector<btVector3> > src_clouds;
			src_clouds.push_back(tseg->ropePts);
			src_clouds.push_back(tseg->boxPts);
			src_clouds.push_back(tseg->holePts);

			warpingDists[i] = getWarpingDistance(src_clouds, target_clouds, u);
		} else {
			warpingDists[i] = numeric_limits<double>::infinity();
		}
	}

	// return the argmin of the warping distance vector.
	return demonums[distance(warpingDists.begin(), min_element(warpingDists.begin(), warpingDists.end()))];
}

/** Changes the lookModes variable to contain the look-information
 *  (which point-clouds to use for registration), for the given demo file number.*/
void ScenePlayer::loadDemoLookInfo(const int demonum) {
	ifstream inpfile(demoinfofname.c_str());
	bool foundInfo = false;

	if(!inpfile.is_open()) {
		cout << "[ERROR : ScenePlayer] : Unable to open demo info file : " << demoinfofname << endl;
		exit(-1);
	} else {
		while(!inpfile.eof()) {
			string line;
			getline(inpfile,line);
			vector<string> splitline;
			string buf;
			stringstream ss(line);

			while (ss >> buf)  // extract white-space separated strings on a line
				splitline.push_back(buf);

			if (splitline.size()==0 || splitline[0][0] == '#') { // skip blank lines and comments
				continue;
			} else {
				if(splitline[1] != ":") {
					cout << "[ERROR : ScenePlayer] : Unknown demo-info file format."<< endl;
					exit(-1);
				}
				int fdemonum = atoi(splitline[0].c_str());
				if (fdemonum == demonum) {
					foundInfo = true;
					lookModes.clear();
					for(int i=2; i < splitline.size(); i+=1) {
						int look_num = atoi(splitline[i].c_str());
						lookModes.push_back(pointModes[look_num]);
					}
					break;
				}
			}
		}

		inpfile.close();

		if (not foundInfo) {
			cout << "[ERROR : ScenePlayer] : No demo-info found for demo num : " << demonum << endl;
			exit(-1);
		}
	}
}


void ScenePlayer::resetPlayer() {
	//playing = false;

	unsigned numdemo;
	if (findClosestDemo) {
		cout << colorize("Looking up the closest demo in the demo library ... [takes a while]", "magenta", true) <<endl;
		numdemo = getClosestDemoNum(transform);

		stringstream msg;
		msg << "Closest demo to current situation found is demo # " << numdemo;
		cout << colorize(msg.str(), "cyan", true) <<endl;
	} else {
		if (RavenConfig::autoLFD and RavenConfig::playnum != -1)
			numdemo = RavenConfig::playnum;
		else
			numdemo= getCurrentPlayNumber();
	}

	// load the look info for all the segments of the demo.
	loadDemoLookInfo(numdemo);

	stringstream scenefnamess;
	scenefnamess << demodir << "/run" << numdemo << ".txt";
	scenefname = scenefnamess.str();

	// create a new trajectory-segmenter:
	tsegmenter.reset(new Segmenter(scenefname));

	currentTimeStampIndex = 0;
	currentGripperActionIndex = 0;

	segNum = -1;

	playTimeStamps.clear();
	currentTrajSeg.reset();
	rjoints.clear();
}

/** Generate the time-stamps for joints. */
void ScenePlayer::genTimeStamps(double startt, double endt, vector<double> &tstamps) {
	tstamps.clear();
	double laststamp = startt;
	do {
		tstamps.push_back(laststamp);
		laststamp += tp;
	} while(laststamp <= endt);
	if (laststamp < endt)
		tstamps.push_back(endt);
}

ScenePlayer::ScenePlayer(CustomScene & _scene, float _freq, bool _doLFD, bool _findClosestDemo, int numfile) :
									scene(_scene),
									freq(_freq),
									doLFD(_doLFD),
									playing(false),
									findClosestDemo(_findClosestDemo),
									segNum(-1),
									currentTimeStampIndex(-1.),
									demodir(EXPAND(RAVENS_DEMO_DIR)),
									runnumfname(string(EXPAND(BULLETSIM_SRC_DIR)) + "/tests/ravens/recorded/playrunnum.txt"),
									demolibfname(string(EXPAND(BULLETSIM_SRC_DIR)) + "/tests/ravens/recorded/demolib.txt"),
									demoinfofname(string(EXPAND(BULLETSIM_SRC_DIR)) + "/tests/ravens/recorded/demo_info.txt")	{

	// set the play-back frequency
	if (freq < 0)
		freq = 100.;
	tp  = 1./freq;

	// the dofs of the robot which are set
	larm_inds = scene.ravens.manipL->manip->GetArmIndices();
	rarm_inds = scene.ravens.manipR->manip->GetArmIndices();

	// register a callback with the scene's step
	scene.addPreStepCallback(boost::bind(&ScenePlayer::playCallback, this));
}

// do LFD based warping or not
void ScenePlayer::toggleLFD() {
	doLFD = not doLFD;
}

// play the demo or not
void ScenePlayer::togglePlay() {
	playing = not playing;

	if (playing) {
		resetPlayer();
		cout << colorize("Now playing demo file: " + scenefname, "green", true)<<endl;
	} else {
		cout << colorize("Stopped playing demo file: " + scenefname, "red", true)<<endl;
	}
}


bool inline ScenePlayer::doNextSegment() {
	return (playTimeStamps.size() ==0 or currentTimeStampIndex==playTimeStamps.size());
}

void ScenePlayer::setupNewSegment() {
	currentTrajSeg = tsegmenter->getNextSegment();

	// stop play-back if no more segments
	if (not currentTrajSeg) {
		playing = false;
		cout << colorize("Done playing demo file: " + scenefname, "green", true)<<endl;

		if (RavenConfig::autoLFD) {
			scene.sceneRecorder->toggleRecording();

			if (RavenConfig::saveImage) {
				stringstream image_fname;
				image_fname << EXPAND(RAVENS_STORE_DIR) << "/run" << scene.sceneRecorder->current_runnum << "-image.jpg";
				scene.captureScene(image_fname.str());
			}

			scene.stopLoop();
		}

		return;
	}

	segNum += 1;
	stringstream msg;
	msg << "Playing a new segment : " <<segNum;
	cout << colorize(msg.str(), "yellow", true) << endl;

	// store the "look" command if auto-recording:
	// for the 0th segment, the scene recorded automatically inserts the look command
	if (RavenConfig::autoLFD and not (segNum == 0)) {
		scene.recordMessage("look\n");
	}

	// sample joints at the correct freqeuncy.
	double tstart;
	if (currentTrajSeg->gtimes.size() != 0)
		tstart = min(currentTrajSeg->jtimes[0], currentTrajSeg->gtimes[0]);
	else
		tstart = currentTrajSeg->jtimes[0];

	double tend;
	if (currentTrajSeg->gtimes.size() != 0)
		tend = max(currentTrajSeg->jtimes.back(), currentTrajSeg->gtimes.back());
	else
		tend = currentTrajSeg->jtimes.back();

	genTimeStamps(tstart, tend, playTimeStamps);

	currentTimeStampIndex = 0;
	currentGripperActionIndex = 0;
	playbackStartTime = scene.getSimTime();


	if (doLFD) {// warp joints.
		vector<vector<btVector3> >  src_clouds;

		if (segNum >= lookModes.size()) {
			cout << "[ERROR : ScenePlayer] : Not enough demo-look info. Exiting." <<endl;
			exit(-1);
		}

		bool use_rope = lookModes[segNum].use_rope;
		bool use_box  = lookModes[segNum].use_box;
		bool use_hole = lookModes[segNum].use_hole;

		// get source clouds
		if (use_rope)
			src_clouds.push_back(currentTrajSeg->ropePts);
		if (use_box)
			src_clouds.push_back(currentTrajSeg->boxPts);
		if (use_hole)
			src_clouds.push_back(currentTrajSeg->holePts);

		// get target clouds
		vector<vector<btVector3> > target_clouds;
		if (use_rope) {
			vector<btVector3> target_rope;
			scene.sRope->getRopePoints(true, target_rope, 1.0/METERS);
			target_clouds.push_back(target_rope);
		}

		if (use_box) {
			vector<btVector3> target_box;
			scene.getBoxPoints(target_box, 1.0/METERS);
			target_clouds.push_back(target_box);
		}

		if (use_hole) {
			vector<btVector3> target_hole;
			scene.getBoxHoles(target_hole, 1.0/METERS);
			target_clouds.push_back(target_hole);
		}

		// Export the position of the suturing pad
		std::pair<SceneGeometry, SceneGeometry> geometry = scene.getSceneGeometry(true);
		std::ofstream out1("/home/pcm/Dropbox/data/1.off");
		out1 << "OFF";
		out1 << geometry.first;
		out1.close();

		std::ofstream out2("/home/pcm/Dropbox/data/2.off");
		out2 << "OFF";
		out2 << geometry.second;
		out2.close();

		SceneGeometry first = load("/home/pcm/Dropbox/data/1.off");
		SceneGeometry second = load("/home/pcm/Dropbox/data/2.off");

	    tetgenio tet = constructMesh(first, second, -15.0, -15.0, 5.0, 15.0, 15.0, 30.0);

	    dolfin::Mesh mesh;
	    dolfin::MeshFunction<std::size_t> ignore(mesh, 2);
	    build_mesh(tet, mesh, ignore);

	    dolfin::plot(mesh, "mesh of the new situation");
	    dolfin::interactive(true);

	    SceneGeometry first_standard = first;//load("/home/pcm/Dropbox/data/standard-geometry-cloth-1.off");
	    SceneGeometry second_standard = second;//load("/home/pcm/Dropbox/data/standard-geometry-cloth-2.off");

	    // YOUR CODE HERE
	    using namespace dolfin;

	    double xmin = -15.0;
	    double ymin = -15.0;
	    double zmin = 5.0;
	    double xmax = 15.0;
	    double ymax = 15.0;
	    double zmax = 30.0;

	    Mesh standard_mesh;

	    dolfin::MeshFunction<std::size_t> boundary(mesh, 2);
	    tetgenio tet_standard = constructMesh(first_standard, second_standard, xmin, ymin, zmin, xmax, ymax, zmax);
	    build_mesh(tet_standard, standard_mesh, boundary);

	    std::cout << "second mesh built" << std::endl;

	    dolfin::plot(standard_mesh, "mesh of standard situation");
	    dolfin::interactive(true);

	    dolfin::plot(boundary, "mesh function");
	    dolfin::interactive(true);

	    // SceneGeometry box = from_tetwrap(make_outer_surface(xmin, ymin, zmin, xmax, ymax, zmax));

	    // parameters["allow_extrapolation"] = true;

	    model::FunctionSpace V(standard_mesh);

	    ObjectToObject left_o2o(btTransform::getIdentity());
	    ObjectToObject outer_o2o(btTransform::getIdentity());
	    //ObjectToObject right_o2o(second.get_transform());
	    ObjectToObject right_o2o(btTransform::getIdentity());

	    // Create Dirichlet boundary conditions
	      DirichletBC bci1(V, left_o2o, boundary, 2);
	      DirichletBC bci2(V, right_o2o, boundary, 3);
	      DirichletBC bco(V, outer_o2o, boundary, 4);
	      std::vector<const DirichletBC*> bcs;
	      bcs.push_back(&bci1);
	      bcs.push_back(&bci2);
	      bcs.push_back(&bco);

	      // Define source and boundary traction functions
	      Constant B(0.0, -0.5, 0.0);
	      Constant T(0.1,  0.0, 0.0);

	      // Define solution function
	      Function* u = new Function(V); // TODO Remove memory leak

	      // Set material parameters
	      const double E  = 10.0;
	      const double nu = 0.3;
	      Constant mu(E/(2*(1 + nu)));
	      Constant lambda(E*nu/((1 + nu)*(1 - 2*nu)));

	      // Create (linear) form defining (nonlinear) variational problem
	      model::ResidualForm F(V);
	      F.mu = mu; F.lmbda = lambda; F.B = B; F.T = T;
	      F.u = *u;

	      // Create jacobian dF = F' (for use in nonlinear solver).
	      model::JacobianForm J(V, V);
	      J.mu = mu; J.lmbda = lambda;
	      J.u = *u;

	      std::cout << "start solving the system" << std::endl;

	      // Solve nonlinear variational problem F(u; v) = 0
	      solve(F == 0, *u, bcs, J);

//std::cout << u[0](10, 10, 10) << " "
	    		  //<< u[1](10, 10, 10) << " "<< u[2](10, 10, 10) << " "
	    		  //<< std::endl;
	      //std::cout << u[0](5, 5, 15) << " "
	      	    		  //<< u[1](5, 5, 15) << " "<< u[2](5, 5, 15) << " "
	      	    		  //<< std::endl;
	      //std::cout << u[0](xmax-0.1, ymax-0.1, zmax-0.1) << " "
	    		  //<< u[1](xmax-0.1, ymax-0.1, zmax-0.1) << " "
	    		  //<< u[1](xmax-0.1, ymax-0.1, zmax-0.1) << " " << std::endl;

	      transform = u;

		// warp the joints using LFD/ Trajopt
		vector<vector<double> > warpedJoints;
		warpRavenJointsBij(scene.ravens, src_clouds, target_clouds,
				currentTrajSeg->joints, warpedJoints,
				lookModes.size(),
				scene.perturbation_vector,
				scene.sceneRecorder->currentSceneFile, transform);

		// interpolate the warped-joints at the play-backtimes
		rjoints = interpolateD( playTimeStamps, warpedJoints, currentTrajSeg->jtimes);

	} else {// just interpolate the recorded joints at the play-back time stamps
		rjoints = interpolateD(playTimeStamps, currentTrajSeg->joints, currentTrajSeg->jtimes);
	}
}


void extractJoints (const vector<int> &inds,
		const vector<dReal> &in_joint_vals, vector<dReal> &out_joint_vals) {
	out_joint_vals.clear();
	out_joint_vals.reserve(inds.size());
	for(int i=0; i<inds.size(); i+=1)
		out_joint_vals.push_back(in_joint_vals[inds[i]]);
}


// callback for playing back joints on the robot.
void ScenePlayer::playCallback() {
	if (playing) {

		if (doNextSegment()) {
			setupNewSegment();
			if (!playing) // startNewSegment can also set playing to false.
				return;
		}

		double playTimeSinceStart = playTimeStamps[currentTimeStampIndex] - playTimeStamps[0];
		double simTimeSinceStart  = scene.getSimTime() - playbackStartTime;

		if (playTimeSinceStart >= simTimeSinceStart) {

			double timestamp = playTimeStamps[currentTimeStampIndex];

			// set the robot-arms joints
			vector<double> larm_joints, rarm_joints;
			extractJoints(larm_inds, rjoints[currentTimeStampIndex], larm_joints);
			extractJoints(rarm_inds, rjoints[currentTimeStampIndex], rarm_joints);

			if (scene.playing) {
				scene.ravens.setArmJointAngles(larm_joints, 'l');
				scene.ravens.setArmJointAngles(rarm_joints, 'r');
			} else{
				cout << "setting joints"<<endl;
			}

			currentTimeStampIndex += 1;

			// set the robot gripper actions
			if (currentGripperActionIndex < currentTrajSeg->grips.size() and currentTrajSeg->gtimes[currentGripperActionIndex] <= timestamp) {
				GripperAction gaction = currentTrajSeg->grips[currentGripperActionIndex];

				currentGripperActionIndex += 1;

				if (gaction == RELEASE_R)
					scene.openGrippers("r");
				else if (gaction == RELEASE_L)
					scene.openGrippers("l");
				else if (gaction == GRAB_R)
					scene.closeGrippers("r");
				else if (gaction == GRAB_L)
					scene.closeGrippers("l");
			}
		}
	}
}
