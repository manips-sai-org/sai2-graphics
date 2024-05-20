// This example application loads a URDF world file with a pendulum and a cube,
// and updates their position continuously

#include <iostream>
#include <string>

#include "Sai2Graphics.h"

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/02-update_rendering/world.urdf";
const string robot_name = "RBot";
const string object_name = "Box";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics =
		new Sai2Graphics::Sai2Graphics(world_file, "sai2 world", true);

	// robot joint pose
	Eigen::VectorXd robot_q = graphics->getRobotJointPos(robot_name);

	// object position and orientation
	Eigen::Affine3d object_pose = graphics->getObjectPose(object_name);

	unsigned long long counter = 0;

	cout << endl
		 << "This example parses a world file containing a pedulum and a cube, "
			"and updates their positions"
		 << endl;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update robot position
		robot_q << (double)counter / 100.0;

		// update object position
		object_pose.translation()(1) = -0.4 * sin((double)counter / 100);
		object_pose.linear() *=
			AngleAxisd(1.0 / 100.0, Eigen::Vector3d::UnitX())
				.toRotationMatrix();

		// update graphics robot and object poses in graphics and render
		graphics->updateRobotGraphics(robot_name, robot_q);
		graphics->updateObjectGraphics(object_name, object_pose);
		graphics->renderGraphicsWorld();

		if(counter == 500)
		{
			cout << "\ndisabling rendering for the object" << endl;
			graphics->setRenderingEnabled(false, object_name);
		}

		if(counter == 800)
		{
			cout << "\nre enabling rendering for the object" << endl;
			graphics->setRenderingEnabled(true, object_name);
		}

		counter++;
	}

	return 0;
}
