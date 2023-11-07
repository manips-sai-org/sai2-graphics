#ifndef SAI2GRAPHICS_UIFORCE_WIDGET_H
#define SAI2GRAPHICS_UIFORCE_WIDGET_H

#include <Sai2Model.h>
#include <chai3d.h>

#include <Eigen/Core>
#include <string>

#include "chai_extension/CRobotLink.h"

namespace Sai2Graphics {

class UIForceWidget {
public:
	enum UIForceWidgetState { Disabled = 0, Inactive, Active };

public:
	// ctor
	UIForceWidget(const std::string &robot_name,
				  std::shared_ptr<Sai2Model::Sai2Model> robot,
				  chai3d::cShapeLine *display_line);

	UIForceWidget(const std::string &object_name,
				  std::shared_ptr<Eigen::Affine3d> object_pose,
				  std::shared_ptr<Eigen::Vector6d> object_velocity,
				  chai3d::cShapeLine *display_line);

	// set state
	void setEnable(bool enable);

	void setNominalSpringParameters(const double linear_stiffness,
									const double rotational_stiffness,
									const double linear_damping,
									const double rotational_damping) {
		_linear_stiffness = linear_stiffness;
		_rotational_stiffness = rotational_stiffness;
		_linear_damping = linear_damping;
		_rotational_damping = rotational_damping;
	}

	// get state
	UIForceWidgetState getState() const { return _state; }

	// set current window and cursor properties this updates the
	// internal parameters for calculating the ui interaction force
	bool setInteractionParams(chai3d::cCamera *camera, int viewx, int viewy,
							  int window_width, int window_height, double depth_change);

	// setter and getter for the mode (force vs moment)
	void setForceMode();
	void setMomentMode();
	bool isForceMode() const { return _force_mode; }

	// get interaction force/moment
	Eigen::Vector6d getAppliedForceMoment() const;

	// get interaction joint torques
	Eigen::VectorXd getUIJointTorques() const;

	const std::string getRobotOrObjectName() { return _robot_or_object_name; }

	// dtor:
	~UIForceWidget() {}

private:
	/**
	 * @brief Get info about link of the robot at the given cursor position.
	 * @return True if a link is present, False if not
	 * @param camera_name Camera name.
	 * @param robot_name Name of robot to look for.
	 * @param view_x x-position of cursor in viewport (OpenGL style screen
	 * co-ordinates).
	 * @param view_y y-position of cursor in viewport (OpenGL style screen
	 * co-ordinates).
	 * @param window_width Width of viewport in screen co-ordinates.
	 * @param window_height Height of viewport in screen co-ordinates.
	 * @param ret_link_name Name of the link. Garbage if no link present at
	 * cursor location.
	 * @param ret_pos Position of cursor in link frame. Garbage if no link
	 * present at cursor location.
	 */
	bool getRobotLinkInCamera(chai3d::cCamera *camera, int view_x, int view_y,
							  int window_width, int window_height,
							  std::string &ret_link_name,
							  Eigen::Vector3d &ret_pos);

	// a line to be displayed when an interaction force is applied
	chai3d::cShapeLine *_display_line;

	// name of robot
	std::string _robot_or_object_name;

	// robot model this UIForceWidget is associated with
	std::shared_ptr<Sai2Model::Sai2Model> _robot;
	std::shared_ptr<Eigen::Affine3d> _object_pose;
	std::shared_ptr<Eigen::Vector6d> _object_velocity;
	bool _is_robot;

	// current state of the widget
	UIForceWidgetState _state;

	// spring constant to use to calculate force/moment
	double _linear_stiffness;
	double _rotational_stiffness;
	double _linear_damping;
	double _rotational_damping;

	// maximum allowable force/moment
	double _max_force;
	double _max_moment;

	// flag to know if we want to apply a force or moment
	bool _force_mode;

	// link on which force is being applied
	std::string _link_name;

	// local position at which force is being applied
	Eigen::Vector3d _link_local_pos;

	// initial poisition of the point that was clicked
	Eigen::Vector3d _initial_click_point;
	double _click_depth;
};

}  // namespace Sai2Graphics

#endif	// SAI2GRAPHICS_UIFORCE_WIDGET_H