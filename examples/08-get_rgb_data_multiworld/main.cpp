#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include "Sai2Graphics.h"
#include "graphics_extension/MultiWorldView.h"

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_fname = "resources/sphbot.urdf";
const string robot_name = "SPHBot";
const string camera_name = "camera1";
const string object_name = "Box";

// save binary file timings
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load robot
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_fname);

	// load graphics scene
	std::vector<std::string> world_file_vec {world_file, world_file};
	std::vector<std::string> camera_name_vec {"camera1", "camera2"};
	auto multi_graphics = std::make_shared<Sai2Graphics::MultiWorldView>(world_file_vec, camera_name_vec, false, "sai2 multi-world");

	// add camera frame buffers for recording 
	multi_graphics->getGraphics(0)->addFrameBuffer("camera1", 96, 96);

	unsigned long long counter = 0;

	cout << endl
		 << "A single robot but with several cameras.\nPress the N key to "
			"switch to the next camera and the B key to switch to the previous "
			"one"
		 << endl;

	auto t1 = high_resolution_clock::now();
	auto t2 = high_resolution_clock::now();
	duration<double, std::milli> ms_double = t2 - t1;

	// while window is open:
	while (multi_graphics->isWindowOpen()) {
		// update graphics robot and object poses in graphics and render
		VectorXd robot_q = VectorXd::Random(robot->q().size()).normalized();
		robot->updateKinematics();
		multi_graphics->getGraphics(0)->updateRobotGraphics(robot_name, robot_q);

		{
			if (counter % 10 == 0) {
				std::cout << "Reset world\n";
				multi_graphics->resetWorld();
				multi_graphics->getGraphics(0)->addFrameBuffer("camera1", 96, 96);
			}
			multi_graphics->renderGraphicsWorld();
			t1 = high_resolution_clock::now();
				// graphics->saveFrameBuffer("camera1", "image1.png");
				// graphics->saveFrameBuffer("camera2", "image2.png");
				// graphics->saveFrameBuffer("camera3", "image3.png");
				// graphics->saveFrameBuffer("camera4", "image4.png");
				multi_graphics->getGraphics(0)->writeFrameBuffer("camera1", "image1");
			// }
			t2 = high_resolution_clock::now();
			ms_double = t2 - t1;
			std::cout << "Duration: " << ms_double.count() << "ms \n";
		}

		counter++;
	}

	return 0;
}
