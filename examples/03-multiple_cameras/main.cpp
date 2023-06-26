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

	cout << endl
		 << "A single robot but with several cameras.\nPress the N key to "
			"switch to the next camera and the B key to switch to the previous "
			"one"
		 << endl;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update graphics robot and object poses in graphics and render
		graphics->renderGraphicsWorld();

		counter++;
	}

	return 0;
}
