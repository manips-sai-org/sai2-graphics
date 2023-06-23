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

	// set state
	void setEnable(bool enable);

	// get state
	UIForceWidgetState getState() const { return _state; }

	// set current window and cursor properties this updates the
	// internal parameters for calculating the ui interaction force
	bool setInteractionParams(chai3d::cCamera *camera, int viewx, int viewy,
							  int window_width, int window_height);

	// get interaction force/moment
	Eigen::Vector3d getUIForceOrMoment(const bool is_force) const;

	// get interaction joint torques
	Eigen::VectorXd getUIJointTorques(const bool is_force_applied) const;

	const std::string getRobotName() { return _robot_name; }

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
	std::string _robot_name;

	// robot model this UIForceWidget is associated with
	std::shared_ptr<Sai2Model::Sai2Model> _robot;

	// handle to graphics interface to query interaction state change
	// Sai2Graphics* _graphics;

	// current state of the widget
	UIForceWidgetState _state;

	// spring constant to use to calculate force/moment
	double _lin_spring_k;
	double _rot_spring_k;

	// maximum allowable force/moment
	double _max_force;
	double _max_moment;

	// link on which force is being applied
	std::string _link_name;

	// local position at which force is being applied
	Eigen::Vector3d _link_local_pos;

	// initial poisition of the point that was clicked
	Eigen::Vector3d _initial_click_point;
};

}  // namespace Sai2Graphics

#endif	// SAI2GRAPHICS_UIFORCE_WIDGET_H