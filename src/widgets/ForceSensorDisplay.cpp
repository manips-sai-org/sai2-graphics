#include "ForceSensorDisplay.h"

namespace Sai2Graphics {

ForceSensorDisplay::ForceSensorDisplay(
	const std::string& robot_name, const std::string& link_name,
	const Eigen::Affine3d T_link_sensor,
	std::shared_ptr<Sai2Model::Sai2Model> robot, chai3d::cWorld* chai_world)
	: _robot(robot),
	  _robot_name(robot_name),
	  _link_name(link_name),
	  _T_link_sensor(T_link_sensor) {
	// initialize display lines
	_display_line_force = new chai3d::cShapeLine();
	_display_line_force->setShowEnabled(false);
	_display_line_force->m_colorPointA.setGreenYellowGreen();
	_display_line_force->m_colorPointB.setGreenYellowGreen();
	chai_world->addChild(_display_line_force);
	_display_line_force->setLineWidth(4.0);

	_display_line_moment = new chai3d::cShapeLine();
	_display_line_moment->setShowEnabled(false);
	_display_line_moment->m_colorPointA.setBrownMaroon();
	_display_line_moment->m_colorPointB.setBrownMaroon();
	chai_world->addChild(_display_line_moment);
	_display_line_moment->setLineWidth(4.0);

	// initialize scales
	_force_line_scale = 0.02;
	_moment_line_scale = 0.1;
}

void ForceSensorDisplay::update(const Eigen::Vector3d& force_global_frame,
								const Eigen::Vector3d& moment_global_frame) {
	Eigen::Vector3d epointA;
	_robot->positionInWorld(epointA, _link_name, _T_link_sensor.translation());

	// force:
	_display_line_force->m_pointA = chai3d::cVector3d(epointA);
	_display_line_force->m_pointB =
		chai3d::cVector3d(epointA - force_global_frame * _force_line_scale);
	_display_line_force->setShowEnabled(true);

	// moment:
	_display_line_moment->m_pointA = chai3d::cVector3d(epointA);
	_display_line_moment->m_pointB =
		chai3d::cVector3d(epointA - moment_global_frame * _moment_line_scale);
	_display_line_moment->setShowEnabled(true);
}

void ForceSensorDisplay::hideLines() {
	_display_line_force->setShowEnabled(false);
	_display_line_force->setShowEnabled(false);
}

}  // namespace Sai2Graphics
