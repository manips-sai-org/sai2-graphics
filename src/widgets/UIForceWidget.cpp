// UIForceWidget.cpp

#include "UIForceWidget.h"

#include <iostream>

using namespace chai3d;
using namespace std;

namespace Sai2Graphics {

UIForceWidget::UIForceWidget(const std::string& robot_name,
                             std::shared_ptr<Sai2Model::Sai2Model> robot,
                             std::shared_ptr<chai3d::cShapeLine> display_line)
    : _robot_name(robot_name), _robot(robot), _display_line(display_line) {
  _display_line->setShowEnabled(false);
  // TODO: set default line display properties

  // set state to inactive initially
  _state = Inactive;

  // TODO: update defaults parameters below
  _max_force = 20;  // N
  _lin_spring_k = 20;   // N/m
  _max_moment = 2;  // Nm
  _rot_spring_k = 2;   // Nm/Rad
}

// enable or disable
void UIForceWidget::setEnable(bool enable) {
  if (_state == Disabled && enable) {
    _state = Inactive;  // we only switch to active when the interaction
                        // paramters are set
  } else if (!enable) {
    _state = Disabled;
    // hide display line
    _display_line->setShowEnabled(false);
  }
}

// set current window and cursor properties
// this updates the internal parameters for calculating the ui interaction force
bool UIForceWidget::setInteractionParams(chai3d::cCamera* camera, int viewx,
                                         int viewy, int window_width,
                                         int window_height) {
  // bool UIForceWidget::setInteractionParams(const bool fLinkSelected, const
  // cCamera* camera)

  // if state is inactive, check if link selection is in progress
  if (_state == Inactive) {
    bool fLinkSelected =
        getRobotLinkInCamera(camera, viewx, viewy, window_width, window_height,
                             _link_name, _link_local_pos);
    if (fLinkSelected) {
      _state = Active;
      _robot->positionInWorld(_initial_click_point, _link_name,
                              _link_local_pos);
      // std::cout << "Active: link " << _link_name << std::endl;
    } else {
      return false;
    }
    // TODO: as an optimization we could perform this check only once per click
  }
  // if state is active,
  if (_state == Active) {
    // update line point A in global graphics frame
    Eigen::Vector3d pointA_pos_base;
    _robot->positionInWorld(pointA_pos_base, _link_name, _link_local_pos);
    _display_line->m_pointA.set(pointA_pos_base[0], pointA_pos_base[1],
                                pointA_pos_base[2]);

    // update line point B. Assumes perspective view!
    // m_fieldViewAngleDeg / 2.0 would correspond to the _top_ of the window
    // cCamera* camera = _graphics->getCamera(camera_name);
    double distCam =
        (window_height / 2.0) / cTanDeg(camera->getFieldViewAngleDeg() / 2.0);

    Eigen::Vector3d selectRay;
    selectRay << -distCam, (viewx - (window_width / 2.0)),
        (viewy - (window_height / 2.0));

    // create a point that's at the same axial distance from the camera as the
    // initial click point Eigen::Vector3d camera_pos, camera_lookat,
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
    cTransform transform = object->getLocalTransform();
    cRobotLink* link;
    while (object_parent != NULL) {
      if (_robot_name == object_parent->m_name) {
        return true;
      }
      if (!f_found_parent_link) {
        // try casting to cRobotLink
        link = dynamic_cast<cRobotLink*>(object_parent);
        if (link != NULL) {
          f_found_parent_link = true;
          ret_link_name = link->m_name;
          // position is with respect to the graphic object. need to go up to
          // the link frame
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
Eigen::Vector3d UIForceWidget::getUIForceOrMoment(const bool is_force) const {
  // nothing to do if state is not active
  if (_state == Disabled || _state == Inactive) {
    return Eigen::Vector3d::Zero();
  }

  double stiffness = is_force ? _lin_spring_k : _rot_spring_k;
  double max = is_force ? _max_force : _max_moment;

  // calculate spring force in global frame
  cVector3d temp = _display_line->m_pointB - _display_line->m_pointA;
  Eigen::Vector3d force_or_moment = temp.eigen();
  force_or_moment = force_or_moment * stiffness;

  // adjust to keep below max force_or_moment
  if (force_or_moment.norm() > max) {
    force_or_moment.normalize();
    force_or_moment = force_or_moment * max;
  }
  return force_or_moment;
}

// get interaction joint torques
Eigen::VectorXd UIForceWidget::getUIJointTorques(const bool is_force_applied) const {
  // nothing to do if state is not active
  if (_state == Disabled || _state == Inactive) {
    return Eigen::VectorXd::Zero(_robot->dof());
  }

  Eigen::Vector3d force_or_moment = getUIForceOrMoment(is_force_applied);

  Eigen::MatrixXd J;
  if(is_force_applied) {
    _robot->Jv(J, _link_name, _link_local_pos);
  } else {
    _robot->Jw(J, _link_name);
  }
  return (J.transpose() * force_or_moment);
}

}  // namespace Sai2Graphics
