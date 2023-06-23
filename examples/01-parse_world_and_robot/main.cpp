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
	auto graphics =
		new Sai2Graphics::Sai2Graphics(world_file, "sai2 world", true);

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update graphics the rendering and the window display.
		// this automatically waits for the correct amount of time
		graphics->updateDisplayedWorld();
	}

	return 0;
}
