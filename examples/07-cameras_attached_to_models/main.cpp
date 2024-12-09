// This example application loads a URDF world file with a pendulum and a cube,
// and updates their position continuously

#include <iostream>
#include <string>

#include "SaiGraphics.h"

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/07-cameras_attached_to_models/world.urdf";
const string robot_name = "RBot";
const string object_name = "Box";

int main() {
	// load graphics scene
	auto graphics = new SaiGraphics::SaiGraphics(world_file);

	// attach camera 2 and 3 to the robot and object respectively
	Matrix3d R_cam_robot;
	R_cam_robot << 0, 0, 1, -1, 0, 0, 0, -1, 0;
	Affine3d T_cam_robot(R_cam_robot);
	T_cam_robot.translation() = Vector3d(0.1, 0.0, 0.0);
	Matrix3d R_cam_object;
	R_cam_object << 0, 0, -1, 1, 0, 0, 0, -1, 0;
	Affine3d T_cam_object(R_cam_object);
	T_cam_object.translation() = Vector3d(-0.1, 0.0, 0.0);
	graphics->attachCameraToRobotLink("camera2", robot_name, "eef",
									  T_cam_robot);
	graphics->attachCameraToObject("camera3", object_name, T_cam_object);

	// robot joint pose
	Eigen::VectorXd robot_q = graphics->getRobotJointPos(robot_name);

	// object position and orientation
	Eigen::Affine3d object_pose = graphics->getObjectPose(object_name);

	unsigned long long counter = 0;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update robot position
		robot_q << (double)counter / 100.0;

		// update object position
		object_pose.translation()(1) = -0.4 * sin((double)counter / 66);
		object_pose.linear() =
			AngleAxisd(M_PI / 4 * sin((double)counter / 150),
					   Eigen::Vector3d::UnitZ())
				.toRotationMatrix();

		// update graphics robot and object poses in graphics and render
		graphics->updateRobotGraphics(robot_name, robot_q);
		graphics->updateObjectGraphics(object_name, object_pose);
		graphics->renderGraphicsWorld();

		counter++;
	}

	return 0;
}
