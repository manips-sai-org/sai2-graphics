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
    auto graphics = new Sai2Graphics::Sai2Graphics(world_file_1);

    // load robot
    auto robot1 = new Sai2Model::Sai2Model(robot_file, false);
    auto robot2 = new Sai2Model::Sai2Model(robot_file, false);

    Eigen::VectorXd q_robot1 = Eigen::VectorXd::Zero(robot1->dof());
    Eigen::VectorXd q_robot2 = Eigen::VectorXd::Zero(robot2->dof());

    unsigned long long counter = 0;
    int current_loaded_world = 0;

    // while window is open:
    while (graphics->isWindowOpen()) {
        // update robot position
        q_robot1(0) += 0.01;
        robot1->set_q(q_robot1);
        robot1->updateKinematics();

        // update graphics rendering and window contents
        graphics->updateGraphics(robot_name_1, robot1);
        if(current_loaded_world == 1) {
            q_robot2(0) -= 0.01;
            robot2->set_q(q_robot2);
            robot2->updateKinematics();
            graphics->updateGraphics(robot_name_2, robot2);
        }
        graphics->updateDisplayedWorld(camera_name);

        if(counter % 700 == 350) {
            graphics->resetWorld(world_file_2);
            q_robot1.setZero();
            q_robot2.setZero();
            current_loaded_world = 1 - current_loaded_world;
        }
        if((counter % 700 == 0) && (counter != 0)) {
            graphics->resetWorld(world_file_1);
            q_robot1.setZero();
            current_loaded_world = 1 - current_loaded_world;
        }
        counter++;
    }

    return 0;
}
