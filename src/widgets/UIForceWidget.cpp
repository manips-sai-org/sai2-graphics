#include "UIForceWidget.h"

#include <iostream>

using namespace chai3d;
using namespace std;

namespace Sai2Graphics {

UIForceWidget::UIForceWidget(const std::string& robot_name,
							 std::shared_ptr<Sai2Model::Sai2Model> robot,
							 chai3d::cShapeLine* display_line)
	: _robot_or_object_name(robot_name),
	  _robot(robot),
	  _display_line(display_line),
	  _is_robot(true) {
	_display_line->m_name = "line_ui_force_" + robot_name;
	_display_line->setShowEnabled(false);

	setForceMode();

	// set state to inactive initially
	_state = Inactive;

	_max_force = 50;  // N
	_max_moment = 5;  // Nm

	setNominalSpringParameters(50, 5, 14, 1);
}

UIForceWidget::UIForceWidget(
	const std::string& object_name,
	std::shared_ptr<Eigen::Affine3d> object_pose,
	std::shared_ptr<Eigen::Matrix<double, 6, 1>> object_velocity,
	chai3d::cShapeLine* display_line)
	: _robot_or_object_name(object_name),
	  _object_pose(object_pose),
	  _object_velocity(object_velocity),
	  _display_line(display_line),
	  _is_robot(false) {
	_display_line->m_name = "line_ui_force_" + object_name;
	_display_line->setShowEnabled(false);

	setForceMode();

	// set state to inactive initially
	_state = Inactive;

	_max_force = 50;  // N
	_max_moment = 5;  // Nm
	setNominalSpringParameters(50, 5, 14, 1);
}

// enable or disable
void UIForceWidget::setEnable(bool enable) {
	if (_state == Disabled && enable) {
		_state = Inactive;	// we only switch to active when the interaction
							// paramters are set
	} else if (!enable) {
		_state = Disabled;
		// hide display line
		_display_line->setShowEnabled(false);
	}
}

void UIForceWidget::setForceMode() {
	_force_mode = true;
	_display_line->m_colorPointA.setGreenYellowGreen();
	_display_line->m_colorPointB.setGreenYellowGreen();
}

void UIForceWidget::setMomentMode() {
	_force_mode = false;
	_display_line->m_colorPointA.setBrownMaroon();
	_display_line->m_colorPointB.setBrownMaroon();
}

// set current window and cursor properties
// this updates the internal parameters for calculating the ui interaction force
bool UIForceWidget::setInteractionParams(chai3d::cCamera* camera, int viewx,
										 int viewy, int window_width,
										 int window_height) {
	// if state is inactive, check if link selection is in progress
	if (_state == Inactive) {
		bool fLinkSelected =
			getRobotLinkInCamera(camera, viewx, viewy, window_width,
								 window_height, _link_name, _link_local_pos);
		if (fLinkSelected) {
			_state = Active;
			_initial_click_point =
				_is_robot ? _robot->positionInWorld(_link_name, _link_local_pos)
						  : *_object_pose * _link_local_pos;
		} else {
			_state = Disabled;
			return false;
		}
	}
	// if state is active,
	if (_state == Active) {
		// update line point A in global graphics frame
		Eigen::Vector3d pointA_pos_base;
		pointA_pos_base =
			_is_robot ? _robot->positionInWorld(_link_name, _link_local_pos)
					  : *_object_pose * _link_local_pos;
		_display_line->m_pointA.set(pointA_pos_base[0], pointA_pos_base[1],
									pointA_pos_base[2]);

		// update line point B. Assumes perspective view!
		// m_fieldViewAngleDeg / 2.0 would correspond to the _top_ of the window
		// cCamera* camera = _graphics->getCamera(camera_name);
		double distCam = (window_height / 2.0) /
						 cTanDeg(camera->getFieldViewAngleDeg() / 2.0);

		Eigen::Vector3d selectRay;
		selectRay << -distCam, (viewx - (window_width / 2.0)),
			(viewy - (window_height / 2.0));

		// create a point that's at the same axial distance from the camera as
		// the initial click point Eigen::Vector3d camera_pos, camera_lookat,
		// camera_vertical; cVector3d pos, vert, lookat;
		Eigen::Vector3d camera_pos = camera->getLocalPos().eigen();
		double lookat_dist = (_initial_click_point - camera_pos).norm();
		selectRay = selectRay * lookat_dist / selectRay.x();
		selectRay = camera->getGlobalRot().eigen() * selectRay;
		_display_line->m_pointB = camera->getGlobalPos() - cVector3d(selectRay);

		// display line
		_display_line->setShowEnabled(true);
	}

	return true;
}

bool UIForceWidget::getRobotLinkInCamera(chai3d::cCamera* camera, int view_x,
										 int view_y, int window_width,
										 int window_height,
										 std::string& ret_link_name,
										 Eigen::Vector3d& ret_pos) {
	cCollisionRecorder selectionRecorder;

	// use standard settings
	cCollisionSettings defaultSettings;
	defaultSettings.m_checkForNearestCollisionOnly = false;
	defaultSettings.m_checkVisibleObjects = true;
	defaultSettings.m_collisionRadius = 0;
	defaultSettings.m_returnMinimalCollisionData = false;

	// use collision detection on the present camera!
	bool hit = camera->selectWorld(view_x, view_y, window_width, window_height,
								   selectionRecorder, defaultSettings);
	if (hit) {
		cVector3d pos = selectionRecorder.m_nearestCollision.m_localPos;
		auto object = selectionRecorder.m_nearestCollision.m_object;
		if (object->getParent() == NULL) {
			return false;
		}
		auto object_parent = object->getParent();
		bool f_found_parent_link = false;
		bool clicked_at_root = true;
		cTransform transform = object->getLocalTransform();
		cRobotLink* link;
		while (object_parent != NULL) {
			if (_robot_or_object_name == object_parent->m_name) {
				if (clicked_at_root) {
					pos = transform * pos;
					ret_pos << pos.x(), pos.y(), pos.z();
				}
				return true;
			}
			if (!f_found_parent_link) {
				// try casting to cRobotLink
				link = dynamic_cast<cRobotLink*>(object_parent);
				if (link != NULL) {
					clicked_at_root = false;
					f_found_parent_link = true;
					ret_link_name = link->m_name;
					// position is with respect to the graphic object. need to
					// go up to the link frame
					pos = transform * pos;
					ret_pos << pos.x(), pos.y(), pos.z();
				} else {
					transform = object_parent->getLocalTransform() * transform;
				}
			}
			object_parent = object_parent->getParent();
		}
	}
	return false;
}

// get interaction force
Eigen::Vector6d UIForceWidget::getAppliedForceMoment() const {
	Eigen::Vector6d force_moment = Eigen::Vector6d::Zero();
	// nothing to do if state is not active
	if (_state == Disabled || _state == Inactive) {
		return force_moment;
	}

	// calculate spring force in global frame
	cVector3d spring_length = _display_line->m_pointB - _display_line->m_pointA;
	if (_force_mode) {
		force_moment.head<3>() = spring_length.eigen() * _linear_stiffness;
	} else {
		force_moment.tail<3>() = spring_length.eigen() * _rotational_stiffness;
	}

	Eigen::Vector6d velocity;
	if (_is_robot) {
		velocity = _robot->velocity6d(_link_name, _link_local_pos);
	} else {
		Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
		J.block<3, 3>(0, 3) = -Sai2Model::crossProductOperator(
			_object_pose->rotation() * _link_local_pos);
		velocity = J * *_object_velocity;
	}

	force_moment.head(3) -= velocity.head(3) * _linear_damping;
	force_moment.tail(3) -= velocity.tail(3) * _rotational_damping;

	// adjust to keep below max force_or_moment
	if (force_moment.head(3).norm() > _max_force) {
		force_moment.head(3) *= _max_force / force_moment.head(3).norm();
	}
	if (force_moment.tail(3).norm() > _max_moment) {
		force_moment.tail(3) *= _max_moment / force_moment.tail(3).norm();
	}

	return force_moment;
}

// get interaction joint torques
Eigen::VectorXd UIForceWidget::getUIJointTorques() const {
	// nothing to do if state is not active
	if (_state == Disabled || _state == Inactive) {
		return _is_robot ? Eigen::VectorXd::Zero(_robot->dof())
						 : Eigen::VectorXd::Zero(6);
	}

	Eigen::Vector6d force_moment = getAppliedForceMoment();

	Eigen::MatrixXd J;
	if (_is_robot) {
		J = _robot->JWorldFrame(_link_name, _link_local_pos);
	} else {
		J = Eigen::MatrixXd::Identity(6, 6);
		J.block<3, 3>(0, 3) = -Sai2Model::crossProductOperator(
			_object_pose->rotation() * _link_local_pos);
	}

	return (J.transpose() * force_moment);
}

}  // namespace Sai2Graphics
