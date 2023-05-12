// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Graphics.h"

#include <iostream>
#include <string>

using namespace std;

const string world_file_1 = "resources/world.urdf";
const string world_file_2 = "resources/world2.urdf";
const string robot_file = "resources/rbot.urdf";
const string robot_name_1 = "RBot";
const string robot_name_2 = "RBot2";
const string camera_name = "camera_fixed";

int main() {
    // load graphics scene
    auto graphics = new Sai2Graphics::Sai2Graphics(world_file_1, true);
    graphics->initializeWindow();

    // load robot
    auto robot1 = new Sai2Model::Sai2Model(robot_file, false);
    robot1->_q.setZero();
    robot1->_dq.setZero();

    auto robot2 = new Sai2Model::Sai2Model(robot_file, false);
    robot2->_q.setZero();
    robot2->_dq.setZero();

    unsigned long long counter = 0;
    int current_loaded_world = 0;

    // while window is open:
    while (graphics->isWindowOpen()) {
        // update robot position
        robot1->_q[0] += 0.01;
        robot1->updateKinematics();

        // update graphics rendering and window contents
        graphics->updateGraphics(robot_name_1, robot1);
        if(current_loaded_world == 1) {
            robot2->_q[0] -= 0.01;
            robot2->updateKinematics();
            graphics->updateGraphics(robot_name_2, robot2);
        }
        graphics->render(camera_name);
        graphics->updateWindowWithCameraView(camera_name);

        if(counter % 700 == 350) {
            graphics->resetWorld(world_file_2);
            robot1->_q[0] = 0;
            robot2->_q[0] = 0;
            current_loaded_world = 1 - current_loaded_world;
        }
        if((counter % 700 == 0) && (counter != 0)) {
            graphics->resetWorld(world_file_1);
            robot1->_q[0] = 0;
            current_loaded_world = 1 - current_loaded_world;
        }
        counter++;
    }

    graphics->closeWindow();

    return 0;
}
