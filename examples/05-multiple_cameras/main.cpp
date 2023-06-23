// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of
// it is also shown using Chai3D.

#include <iostream>
#include <string>

#include "Sai2Graphics.h"

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_name = "RBot";
const string camera_name = "camera_fixed";
const string object_name = "Box";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics =
		new Sai2Graphics::Sai2Graphics(world_file, "sai2 world", true);

	unsigned long long counter = 0;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update graphics robot and object poses in graphics and render
		graphics->updateDisplayedWorld();

		counter++;
	}

	return 0;
}
