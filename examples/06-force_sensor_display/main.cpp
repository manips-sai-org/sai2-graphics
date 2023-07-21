// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of
// it is also shown using Chai3D.

#include <iostream>
#include <string>

#include "Sai2Graphics.h"

using namespace std;

const string world_file = "resources/world.urdf";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file);

	// create force data and force display
	Sai2Model::ForceSensorData force_data = Sai2Model::ForceSensorData();
	force_data.robot_name = "PBot";
	force_data.link_name = "cube_link";
	graphics->addForceSensorDisplay(force_data);

	unsigned long long counter = 0;

	cout << endl
		 << "Sensed force data is rendered in the graphics world. After some "
			"time, a vertical force is rendered, then a tengential moment is "
			"added and finally a tengential force is added"
		 << endl;
	cout << endl;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update graphics robot and object poses in graphics and render
		graphics->updateDisplayedForceSensor(force_data);
		graphics->renderGraphicsWorld();

		if (counter == 200) {
			cout << "display vertical force" << endl;
			force_data.force_world_frame(2) = -10.0;
		}
		if (counter == 400) {
			cout << "display tangential moment" << endl;
			force_data.moment_world_frame(1) = 1.0;
		}
		if (counter == 600) {
			cout << "display tangential and vertical force" << endl;
			force_data.force_world_frame(1) = 5.0;
		}

		counter++;
	}

	return 0;
}
