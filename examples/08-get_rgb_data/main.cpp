#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include "Sai2Graphics.h"

using namespace std;

const string world_file = "resources/world.urdf";
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

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, "sai2 world", true);
	graphics->addFrameBuffer("camera1", 96, 96);
	// graphics->addFrameBuffer("camera2");
	// graphics->addFrameBuffer("camera3");
	// graphics->addFrameBuffer("camera4");

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
	while (graphics->isWindowOpen()) {
		// update graphics robot and object poses in graphics and render
		{
			graphics->renderGraphicsWorld();
			t1 = high_resolution_clock::now();
			// if (counter % 60 == 0) {
				// graphics->saveFrameBuffer("camera1", "image1.png");
				// graphics->saveFrameBuffer("camera2", "image2.png");
				// graphics->saveFrameBuffer("camera3", "image3.png");
				// graphics->saveFrameBuffer("camera4", "image4.png");
				graphics->writeFrameBuffer("camera1", "image1");
			// }
			t2 = high_resolution_clock::now();
			ms_double = t2 - t1;
			std::cout << "Duration: " << ms_double.count() << "ms \n";
		}

		counter++;
	}

	return 0;
}