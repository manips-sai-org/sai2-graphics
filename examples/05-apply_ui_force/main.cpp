// This example shows we can right click on the UI window and drag the mouse
// to generate forces. In this example, we show the line corresponding to the
// forces and get the corresponding joint torques for the robot, to display in
// terminal.

#include <iostream>
#include <string>

#include "Sai2Graphics.h"

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_name = "RBot";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	Eigen::VectorXd q_robot = graphics->getRobotJointPos(robot_name);

	// set up ui force interaction
	graphics->addUIForceInteraction(robot_name);
	Eigen::VectorXd ui_interaction_torques;

	unsigned long long counter = 0;

	cout << endl
		 << "It is possible to interact with the displayed robot using the "
			"right mouse button. A line will be displayed, illustrating the "
			"applied force (or torque) on the robot, and the corresponding "
			"torque on the robot joint will be printed to terminal periodically"
		 << endl;
	cout << "Righy mouse button: apply a force" << endl;
	cout << "Righy mouse button + shift: apply a moment" << endl;
	cout << endl;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update graphics rendering and window contents
		graphics->updateRobotGraphics(robot_name, q_robot);
		graphics->renderGraphicsWorld();
		ui_interaction_torques = graphics->getUITorques(robot_name);

		if (counter % 50 == 0) {
			std::cout << "interaction torques: "
					  << ui_interaction_torques.transpose() << std::endl;
		}

		counter++;
	}

	return 0;
}
