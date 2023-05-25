// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Graphics.h"

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/rbot.urdf";
const string robot_name = "RBot";
const string camera_name = "camera_fixed";
const string object_name = "Box";

int main() {
    cout << "Loading URDF world model file: " << world_file << endl;

    // load graphics scene
    auto graphics = new Sai2Graphics::Sai2Graphics(world_file);

    // load robot
    auto robot = new Sai2Model::Sai2Model(robot_file, false);
    int dof = robot->dof();
    Eigen::VectorXd next_q = robot->q();

    Eigen::Vector3d object_pos = Eigen::Vector3d(0, 0, -1.5);
    Eigen::Matrix3d object_ori = Eigen::Matrix3d::Identity();

    unsigned long long counter = 0;

    // while window is open:
    while (graphics->isWindowOpen()) {
        // update robot position
        next_q << (double) counter/100.0;
        robot->set_q(next_q);
        robot->updateKinematics();

        // update object position
        object_pos(1) = -0.4 * sin( (double) counter/100);
        object_ori *= AngleAxisd( 1.0/100.0, Eigen::Vector3d::UnitX()).toRotationMatrix();

        // update graphics rendering and window contents
        graphics->updateGraphics(robot_name, robot);
        graphics->updateObjectGraphics(object_name, object_pos, Eigen::Quaterniond(object_ori));
        graphics->updateDisplayedWorld(camera_name);

        counter++;
    }

    return 0;
}
