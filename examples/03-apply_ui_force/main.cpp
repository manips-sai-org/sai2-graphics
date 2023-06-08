// This example shows we can right click on the UI window and drag the mouse
// to generate forces. In this example, we show the line corresponding to the forces
// and get the corresponding joint torques for the robot, to display in terminal.

#include "Sai2Graphics.h"

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_name = "RBot";
const string camera_name = "camera_fixed";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file);
    Eigen::VectorXd q_robot = graphics->getRobotJointPos(robot_name);

    // set up ui force interaction
    graphics->addUIForceInteraction(robot_name);
    Eigen::VectorXd ui_interaction_torques;

    unsigned long long counter = 0;

    // while window is open:
    while (graphics->isWindowOpen())
	{
		// update graphics rendering and window contents
        graphics->updateRobotGraphics(robot_name, q_robot);
        graphics->updateDisplayedWorld(camera_name);
        graphics->getUITorques(robot_name, ui_interaction_torques);

        if(counter % 50 == 0) {
            std::cout << "interaction torques: " << ui_interaction_torques.transpose() << std::endl;
        }

        counter++;
	}

	return 0;
}
