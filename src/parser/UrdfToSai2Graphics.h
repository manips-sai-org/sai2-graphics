/**
 * \file UrdfToSai2Graphics.h
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#ifndef URDF_TO_SAI2GRAPHICS_H
#define URDF_TO_SAI2GRAPHICS_H

#include <chai3d.h>

#include "chai_extension/CRobotBase.h"
#include "chai_extension/CRobotLink.h"
#include "chai_extension/Capsule.h"
#include "chai_extension/Pyramid.h"

namespace Parser {
/**
 * @brief Parse a URDF file and populate a chai3d world model from it.
 * @param filename URDF world model file to parse.
 * @param world chai3d::cWorld model to populate from parsed file.
 * @param verbose To display information about the robot model creation in the
 * terminal or not.
 */
void UrdfToSai2GraphicsWorld(
	const std::string& filename, chai3d::cWorld* world,
	std::map<std::string, std::string>& robot_filenames,
	std::map<std::string, std::shared_ptr<Eigen::Affine3d>>& dyn_object_poses,
	std::map<std::string, std::shared_ptr<Eigen::Affine3d>>&
		static_object_poses,
	std::vector<std::string>& camera_names, bool verbose);

/**
 * @brief Parse a URDF file and populate a single chai3d robot model from it.
 * @param filename URDF robot model file to parse.
 * @param model chai3d::cRobotBase model to populate from parsed file.
 * @param verbose To display information about the robot model creation in the
 * terminal or not.
 * @param working_dirname Directory path relative to which paths within the
 * model file are specified.
 */
void UrdfToSai2GraphicsRobot(const std::string& filename,
							 chai3d::cRobotBase* base, bool verbose,
							 const std::string& working_dirname = "./");
// TODO: working dir default should be "", but this requires checking
// to make sure that the directory path has a trailing backslash

}  // namespace Parser

#endif	// URDF_TO_SAI2GRAPHICS_H