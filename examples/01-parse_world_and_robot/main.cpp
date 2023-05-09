// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Graphics.h"

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/world.urdf";
const string camera_name = "camera_fixed";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
    graphics->initializeWindow();

    // while window is open:
    while (graphics->isWindowOpen())
	{
		// update graphics the rendering and the window display. 
        // this automatically waits for the correct amount of time
		graphics->render(camera_name);
        graphics->updateWindowWithCameraView(camera_name);
	}

    graphics->closeWindow();

	return 0;
}
