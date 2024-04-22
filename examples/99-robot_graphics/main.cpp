#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include "Sai2Graphics.h"

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_name = "panda_arm";
const string camera_name = "camera";
using Vector7d = Eigen::Matrix<double, 7, 1>;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

    Vector7d q0, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14, q15;
	q0 << 0, 0, 0, -90, 0, 90, 0;  // initial configuration 
	q1 << 0, 0, 0, -90, 0, 45, 0;
	q2 << 0, 0, 0, -90, 0, 0, 0;
	q3 << 0, 0, 0, -90, 0, 135, 0;
	q4 << 0, 0, 0, -90, 0, 180, 0;

    q5 << 0, 0, 0, -90, -135, 90, 45;
	q6 << 0, 0, 0, -90, -90, 90, 45;  
	q7 << 0, 0, 0, -90, -45, 90, 45;  
	q8 << 0, 0, 0, -90, 0, 90, 45;
	q9 << 0, 0, 0, -90, 45, 90, 45;  
	q10 << 0, 0, 0, -90, 90, 90, 45;  
    q11 << 0, 0, 0, -90, 135, 90, 45; 

	q12 << 0, 0, 0, -90, 45, 45, 45;
	q13 << 0, 0, 0, -90, 45, 135, 45;
	q14 << 0, 0, 0, -90, -45, 45, 45;
    q15 << 0, 0, 0, -90, -45, 135, 45;

	vector<Vector7d> calib_config = { (M_PI / 180) * q0, (M_PI / 180) * q1, (M_PI / 180) * q2, (M_PI / 180) * q3,
										(M_PI / 180) * q4, (M_PI / 180) * q5, (M_PI / 180) * q6, (M_PI / 180) * q7,
										(M_PI / 180) * q8, (M_PI / 180) * q9, (M_PI / 180) * q10, (M_PI / 180) * q11,
										(M_PI / 180) * q12, (M_PI / 180) * q13, (M_PI / 180) * q14, (M_PI / 180) * q15 };

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, "sai2 world", true);
    Affine3d camera_on_robot_pose = Affine3d::Identity();
    camera_on_robot_pose.translation() = Vector3d(0.066, 0, 0.107 + 0.030);
    camera_on_robot_pose.linear() = AngleAxisd(-M_PI / 2, Vector3d::UnitZ()).toRotationMatrix();
    graphics->setCameraFov(camera_name, 65 * M_PI / 180);

    graphics->updateRobotGraphics(robot_name, calib_config[0]);

	unsigned long long counter = 0;

	cout << endl
		 << "A single robot but with several cameras.\nPress the N key to "
			"switch to the next camera and the B key to switch to the previous "
			"one"
		 << endl;

	// while window is open:
    int cnt = 0;
	while (graphics->isWindowOpen()) {
        // set camera to follow the end-effector 
        graphics->setCameraOnRobot(camera_name, robot_name, "link7", camera_on_robot_pose);

		// update graphics robot and object poses in graphics and render
        if (counter % 10 == 0) {
            // graphics->updateRobotGraphics(robot_name, calib_config[cnt++]);
        }
        graphics->renderGraphicsWorld();
		counter++;
	}

	return 0;
}