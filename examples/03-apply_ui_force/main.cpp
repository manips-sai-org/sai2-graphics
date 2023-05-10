// This example shows we can right click on the UI window and drag the mouse
// to generate forces. In this example, we show the line corresponding to the forces
// and get the corresponding joint torques for the robot, to display in terminal.

#include "Sai2Graphics.h"

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/rbot.urdf";
const string robot_name = "RBot";
const string camera_name = "camera_fixed";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
    graphics->initializeWindow();

    // load robot
    auto robot = new Sai2Model::Sai2Model(robot_file, false);
    int dof = robot->dof();
    robot->_q.setZero();
    robot->_dq.setZero();

    // set up ui force interaction
    graphics->addUIForceInteraction(robot_name, robot);
    Eigen::VectorXd ui_interaction_torques = Eigen::VectorXd::Zero(dof);

    unsigned long long counter = 0;

    // while window is open:
    while (graphics->isWindowOpen())
	{
        // update robot position
        robot->updateKinematics();

		// update graphics rendering and window contents
        graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name);
        graphics->updateWindowWithCameraView(camera_name);
        graphics->getUITorques(robot_name, ui_interaction_torques);

        if(counter % 50 == 0) {
            std::cout << "interaction torques: " << ui_interaction_torques.transpose() << std::endl;
        }

        counter++;
	}

    graphics->closeWindow();

	return 0;
}
