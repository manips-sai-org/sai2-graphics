#ifndef SaiGraphics_FORCE_SENSOR_DISPLAY_H
#define SaiGraphics_FORCE_SENSOR_DISPLAY_H

#include <SaiModel.h>
#include <chai3d.h>

namespace SaiGraphics {

// Class to display force and moments sensed by a force/moment sensor
// This will display the reaction forces/moments on the sensor.
// For example, if the sensor is pushing down on the floor, the displayed line
// for the force will go up.
class ForceSensorDisplay {
public:
	ForceSensorDisplay(const std::string& robot_name,
					   const std::string& link_name,
					   const Eigen::Affine3d T_link_sensor,
					   std::shared_ptr<SaiModel::SaiModel> robot,
					   chai3d::cWorld* chai_world);

	ForceSensorDisplay(const std::string& object_name,
					   const std::string& link_name,
					   const Eigen::Affine3d T_link_sensor,
					   std::shared_ptr<Affine3d> object_pose,
					   chai3d::cWorld* chai_world);

	void update(const Eigen::Vector3d& force_global_frame,
				const Eigen::Vector3d& moment_global_frame);

	void hideLines();

	std::string robot_or_object_name() const { return _robot_or_object_name; }
	std::string link_name() const { return _link_name; }
	Eigen::Affine3d T_link_sensor() const { return _T_link_sensor; }

private:
	void initializeLines(chai3d::cWorld* chai_world);

	// a line to be displayed when a contact force is active
	chai3d::cShapeLine* _display_line_force;

	// a line to be displayed when a contact moment is active
	chai3d::cShapeLine* _display_line_moment;

	// robot model
	std::shared_ptr<SaiModel::SaiModel> _robot;
	const std::string _robot_or_object_name;
	const std::string _link_name;
	const Eigen::Affine3d _T_link_sensor;

	// object pose (if the sensor is attached to an object)
	std::shared_ptr<Eigen::Affine3d> _object_pose;

	// scale of the force line displayed from 0 to 1
	double _force_line_scale;

	// scale of the moment line displayed from 0 to 1
	double _moment_line_scale;
};

}  // namespace SaiGraphics

#endif	// SaiGraphics_FORCE_SENSOR_DISPLAY_H
