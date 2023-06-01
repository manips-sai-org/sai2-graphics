/**
 * \file Sai2Graphics.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#include "Sai2Graphics.h"
#include "parser/UrdfToSai2Graphics.h"
#include <iostream>

using namespace std;
using namespace chai3d;

namespace
{

	// flags for scene camera movement
	bool fTransXp = false;
	bool fTransXn = false;
	bool fTransYp = false;
	bool fTransYn = false;
	bool fTransZp = false;
	bool fTransZn = false;
	bool fRotPanTilt = false;
	bool fShowCameraPose = false;
	bool fShowCameraPoseReset = true;
	bool fRobotLinkSelect = false;
	bool fRobotLinkSelectReset = true;
	bool fShiftPressed = false;

	// callback to print glfw errors
	void glfwError(int error, const char *description)
	{
		cerr << "GLFW Error: " << description << endl;
		exit(1);
	}

	// callback when a key is pressed
	void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods)
	{
		bool set = (action != GLFW_RELEASE);
		switch (key)
		{
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		case GLFW_KEY_S:
			fShowCameraPose = set;
			if(!fShowCameraPose) {
				fShowCameraPoseReset = true;
			}
			break;
		case GLFW_KEY_LEFT_SHIFT:
			fShiftPressed = set;
		default:
			break;
		}
	}

	// callback when a mouse button is pressed
	void mouseClick(GLFWwindow *window, int button, int action, int mods)
	{
		bool set = (action != GLFW_RELEASE);
		// TODO: mouse interaction with robot
		switch (button)
		{
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			break;
		// right click to interact with robot by applying forces
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			if(!fRobotLinkSelect) {
				fRobotLinkSelectReset = true;
			}
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
		}
	}

	GLFWwindow *glfwInitialize(const std::string& window_name)
	{
		/*------- Set up visualization -------*/
		// set up error callback
		glfwSetErrorCallback(glfwError);

		// initialize GLFW
		glfwInit();

		// retrieve resolution of computer display and position window accordingly
		GLFWmonitor *primary = glfwGetPrimaryMonitor();
		const GLFWvidmode *mode = glfwGetVideoMode(primary);

		// information about computer screen and GLUT display window
		int screenW = mode->width;
		int screenH = mode->height;
		int windowW = 0.8 * screenH;
		int windowH = 0.5 * screenH;
		int windowPosY = (screenH - windowH) / 2;
		int windowPosX = windowPosY;

		// create window and make it current context
		glfwWindowHint(GLFW_VISIBLE, 0);
		GLFWwindow *window = glfwCreateWindow(windowW, windowH, window_name.c_str(), NULL, NULL);
		glfwSetWindowPos(window, windowPosX, windowPosY);
		glfwShowWindow(window);
		glfwMakeContextCurrent(window);
		glfwSwapInterval(1);

		return window;
	}

}

namespace Sai2Graphics 
{

Sai2Graphics::Sai2Graphics(const std::string& path_to_world_file,
							const std::string& window_name,
							bool verbose)
{
	// initialize a chai world
	initializeWorld(path_to_world_file, verbose);
	initializeWindow(window_name);
}

// dtor
Sai2Graphics::~Sai2Graphics() {
	glfwDestroyWindow(_window);
	glfwTerminate();
	clearWorld();
	_world = NULL;
}

void Sai2Graphics::resetWorld(const std::string& path_to_world_file, const bool verbose) {
	clearWorld();
	initializeWorld(path_to_world_file, verbose);
}

void Sai2Graphics::initializeWorld(const std::string& path_to_world_file, const bool verbose) {
	_world = new chai3d::cWorld();
	Parser::UrdfToSai2GraphicsWorld(path_to_world_file, _world, _robot_filenames, verbose);
	for(auto robot_filename : _robot_filenames) {
		// get robot base object in chai world
		cRobotBase* base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
			if (robot_filename.first == _world->getChild(i)->m_name) {
				// cast to cRobotBase
				base = dynamic_cast<cRobotBase*>(_world->getChild(i));
				if (base != NULL) {
					break;
				}
			}
		}
		Eigen::Affine3d T_robot_base;
		T_robot_base.translation() = base->getLocalPos().eigen();
		T_robot_base.linear() = base->getLocalRot().eigen();
		_robot_models[robot_filename.first] = std::make_shared<Sai2Model::Sai2Model>(robot_filename.second, false, T_robot_base);
	}
}

void Sai2Graphics::clearWorld() {
	delete _world;
	_robot_filenames.clear();
	_robot_models.clear();
}

void Sai2Graphics::initializeWindow(const std::string& window_name) {
	_window = glfwInitialize(window_name);

	// set callbacks
	glfwSetKeyCallback(_window, keySelect);
	glfwSetMouseButtonCallback(_window, mouseClick);
}

void Sai2Graphics::addUIForceInteraction(const std::string& robot_name) {
	auto it = _robot_models.find(robot_name);
	if(it == _robot_models.end()) {
		throw std::invalid_argument("robot not found in Sai2Graphics::addUIForceInteraction");
	}
	for(auto widget : _ui_force_widgets) {
		if(robot_name == widget->getRobotName()) {
			return;
		}
	}
	chai3d::cShapeLine* display_line = new chai3d::cShapeLine();
	_world->addChild(display_line);
	_ui_force_widgets.push_back(std::make_shared<UIForceWidget>(robot_name, _robot_models[robot_name], display_line));
}

void Sai2Graphics::getUITorques(const std::string& robot_name, Eigen::VectorXd& ret_torques) {
	ret_torques.setZero();
	for(auto widget : _ui_force_widgets) {
		if(robot_name == widget->getRobotName()) {
			ret_torques = widget->getUIJointTorques(!fShiftPressed);
			return;
		}
	}
}

void Sai2Graphics::updateDisplayedWorld(const std::string &camera_name) {
	// update graphics. this automatically waits for the correct amount of time
	glfwGetFramebufferSize(_window, &_window_width, &_window_height);
	glfwSwapBuffers(_window);
	glFinish();

	// poll for events
	glfwPollEvents();

	// move scene camera as required
	getCameraPose(camera_name, _camera_pos, _camera_up_axis, _camera_lookat_point);
	Vector3d cam_depth_axis = _camera_lookat_point - _camera_pos;
	cam_depth_axis.normalize();
	// Vector3d cam_up_axis;
	// cam_up_axis << 0.0, 0.0, 1.0; // TODO: there might be a better way to do this
	Vector3d cam_right_axis = cam_depth_axis.cross(_camera_up_axis);
	cam_right_axis.normalize();
	// Vector3d cam_lookat_axis = _camera_lookat;
	// cam_lookat_axis.normalize();
	if (fTransXp)
	{
			_camera_pos += 0.05 * cam_right_axis;
			_camera_lookat_point += 0.05 * cam_right_axis;
	}
	if (fTransXn)
	{
			_camera_pos -= 0.05 * cam_right_axis;
			_camera_lookat_point -= 0.05 * cam_right_axis;
	}
	if (fTransYp)
	{
			_camera_pos += 0.05 * _camera_up_axis;
			_camera_lookat_point += 0.05 * _camera_up_axis;
	}
	if (fTransYn)
	{
			_camera_pos -= 0.05 * _camera_up_axis;
			_camera_lookat_point -= 0.05 * _camera_up_axis;
	}
	if (fTransZp)
	{
			_camera_pos += 0.1 * cam_depth_axis;
			_camera_lookat_point += 0.1 * cam_depth_axis;
	}
	if (fTransZn)
	{
			_camera_pos -= 0.1 * cam_depth_axis;
			_camera_lookat_point -= 0.1 * cam_depth_axis;
	}
	if (fShowCameraPose && fShowCameraPoseReset)
	{
			fShowCameraPoseReset = false;
			cout << endl;
			cout << "camera position : " << _camera_pos.transpose() << endl;
			cout << "camera lookat point : " << _camera_lookat_point.transpose() << endl;
			cout << "camera up axis : " << _camera_up_axis.transpose() << endl;
			cout << endl;
	}
	if (fRotPanTilt)
	{
		// get current cursor position
		double cursorx, cursory;
		glfwGetCursorPos(_window, &cursorx, &cursory);
		// TODO: might need to re-scale from screen units to physical units
		double compass = 0.006 * (cursorx - _last_cursorx);
		double azimuth = 0.006 * (cursory - _last_cursory);
		double radius = cam_depth_axis.norm();
		Matrix3d m_tilt;
		m_tilt = AngleAxisd(azimuth, -cam_right_axis);
		_camera_pos = _camera_lookat_point + m_tilt * (_camera_pos - _camera_lookat_point);
		Matrix3d m_pan;
		m_pan = AngleAxisd(compass, -_camera_up_axis);
		_camera_pos = _camera_lookat_point + m_pan * (_camera_pos - _camera_lookat_point);
		_camera_up_axis = m_pan * _camera_up_axis;
	}
	setCameraPose(camera_name, _camera_pos, _camera_up_axis, _camera_lookat_point);
	glfwGetCursorPos(_window, &_last_cursorx, &_last_cursory);

	// allows the user to drag on a link and apply a force
	if (fRobotLinkSelect) {
		if(fRobotLinkSelectReset) {
			for(auto widget : _ui_force_widgets) {
				widget->setEnable(true);
			}			
			fRobotLinkSelectReset = false;
		}

		int wwidth_scr, wheight_scr;
		glfwGetWindowSize(_window, &wwidth_scr, &wheight_scr);

		int viewx = floor(_last_cursorx / wwidth_scr * _window_width);
		int viewy = floor(_last_cursory / wheight_scr * _window_height);

		for(auto widget : _ui_force_widgets) {
			widget->setInteractionParams(getCamera(camera_name), viewx, _window_height - viewy, _window_width, _window_height);
		}
	} else {
		for(auto widget : _ui_force_widgets) {
			widget->setEnable(false);
		}	
	}
	render(camera_name);
}

static void updateGraphicsLink(cRobotLink* link, std::shared_ptr<Sai2Model::Sai2Model> robot_model) {
	cVector3d local_pos;
	cMatrix3d local_rot;

	// get link name
	std::string link_name = link->m_name;

	// get chai parent name
	cGenericObject* parent = link->getParent();
	std::string parent_name = parent->m_name;

	// if parent is cRobotBase, then simply get transform relative to base from model
	if (dynamic_cast<cRobotBase*> (parent) != NULL) {
		Eigen::Affine3d T;
		robot_model->transform(T, link_name);
		local_pos = cVector3d(T.translation());
		local_rot = cMatrix3d(T.rotation());
	} else if (dynamic_cast<cRobotLink*> (parent) != NULL) {
		// if parent is cRobotLink, then calculate transform for both links, then apply inverse transform
		Eigen::Affine3d T_me, T_parent, T_rel;
		robot_model->transform(T_me, link_name);
		robot_model->transform(T_parent, parent_name);
		T_rel = T_parent.inverse() * T_me;
		local_pos = cVector3d(T_rel.translation());
		local_rot = cMatrix3d(T_rel.rotation());
	} else {
		cerr << "Parent to link " << link_name << " is neither link nor base" << endl;
		abort();
		// TODO: throw exception
	}
	link->setLocalPos(local_pos);
	link->setLocalRot(local_rot);

	// call on children
	cRobotLink* child;
	for (unsigned int i = 0; i < link->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(link->getChild(i));
		if (child != NULL) {
			updateGraphicsLink(child, robot_model);
		}
	}
}

// update frame for a particular robot
void Sai2Graphics::updateRobotGraphics(const std::string& robot_name,
									const Eigen::VectorXd& joint_angles) {
	// update corresponfing robot model
	auto it = _robot_models.find(robot_name);
	if(it == _robot_models.end()) {
		throw std::invalid_argument("Robot not found in Sai2Graphics::updateRobotGraphics");
	}
	auto robot_model = _robot_models[robot_name];
	if(joint_angles.size() != robot_model->q_size()) {
		throw std::invalid_argument("size of joint angles inconsistent with robot model in Sai2Graphics::updateRobotGraphics");
	}
	robot_model->set_q(joint_angles);
	robot_model->updateKinematics();
	
	// get robot base object in chai world
	cRobotBase* base = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (robot_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			base = dynamic_cast<cRobotBase*>(_world->getChild(i));
			if (base != NULL) {
				break;
			}
		}
	}
	if (base == NULL) {
		//TODO: throw exception
		cerr << "Could not find robot in chai world: " << robot_name << endl;
		abort();
	}
	// recursively update graphics for all children
	cRobotLink* link;
	for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
		link = dynamic_cast<cRobotLink*>(base->getChild(i));
		if (link != NULL) {
			updateGraphicsLink(link, robot_model);
		}
	}
	// update shadow maps. TODO: consider moving out from here if it is too expensive
	_world->updateShadowMaps();
}

void Sai2Graphics::updateObjectGraphics(const std::string& object_name,
                         const Eigen::Vector3d& object_pos,
                         const Eigen::Quaterniond object_ori)
{
	cGenericObject* object = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (object_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			object = _world->getChild(i);
			if (object != NULL) {
				break;
			}
		}
	}
	if (object == NULL) {
		//TODO: throw exception
		cerr << "Could not find object in chai world: " << object_name << endl;
		abort();
	}

	// update pose
	object->setLocalPos(object_pos);
	object->setLocalRot(object_ori.toRotationMatrix());
}

Eigen::VectorXd Sai2Graphics::getRobotJointPos(const std::string& robot_name) {
	auto it = _robot_models.find(robot_name);
	if(it == _robot_models.end()) {
		throw std::invalid_argument("robot not found in Sai2Graphics::getRobotJointPos");
	}
	return _robot_models[robot_name]->q();
}


void Sai2Graphics::render(const std::string& camera_name) {
	auto camera = getCamera(camera_name);
	// TODO: support link mounted cameras
	// TODO: support stereo. see cCamera::renderView
	//	to do so, we need to search through the descendent tree
	// render view from this camera
	// NOTE: we don't use the display context id right now since chai no longer
	// supports it in 3.2.0
	camera->renderView(_window_width, _window_height);
}

// get current camera pose
void Sai2Graphics::getCameraPose(const std::string& camera_name,
									Eigen::Vector3d& ret_position,
									Eigen::Vector3d& ret_vertical_axis,
									Eigen::Vector3d& ret_lookat_point) {
	auto camera = getCamera(camera_name);
	cVector3d pos, vert, lookat;
	pos = camera->getLocalPos(); ret_position << pos.x(), pos.y(), pos.z();
	vert = camera->getUpVector(); ret_vertical_axis << vert.x(), vert.y(), vert.z();
	lookat = camera->getLookVector(); ret_lookat_point << lookat.x(), lookat.y(), lookat.z();
	ret_lookat_point += ret_position;
}

// set camera pose
void Sai2Graphics::setCameraPose(const std::string& camera_name,
									const Eigen::Vector3d& position,
									const Eigen::Vector3d& vertical_axis,
									const Eigen::Vector3d& lookat_point) {
 	auto camera = getCamera(camera_name);
	cVector3d pos(position[0], position[1], position[2]);
	cVector3d vert(vertical_axis[0], vertical_axis[1], vertical_axis[2]);
	cVector3d look(lookat_point[0], lookat_point[1], lookat_point[2]);
	camera->set(pos, look, vert);
}

bool Sai2Graphics::getRobotLinkInCamera(const std::string& camera_name,
		                                   const std::string& robot_name,
		                                   int view_x,
		                                   int view_y,
		                                   int window_width,
										   int window_height,
		                                   std::string& ret_link_name,
		                                   Eigen::Vector3d& ret_pos) {
	cCollisionRecorder selectionRecorder;

	//use standard settings
	cCollisionSettings defaultSettings;
	defaultSettings.m_checkForNearestCollisionOnly = false;
	defaultSettings.m_checkVisibleObjects = true;
	defaultSettings.m_collisionRadius = 0;
	defaultSettings.m_returnMinimalCollisionData = false;

	//use collision detection on the present camera!
	bool hit = getCamera(camera_name)->selectWorld(
										view_x,
										view_y,
										window_width,
										window_height,
										selectionRecorder,
										defaultSettings);
	if(hit) {
		cVector3d pos = selectionRecorder.m_nearestCollision.m_localPos;
		auto object = selectionRecorder.m_nearestCollision.m_object;
		if(object->getParent() == NULL) {
			return false;
		}
		auto object_parent = object->getParent();
		bool f_found_parent_link = false;
		cTransform transform = object->getLocalTransform();
		cRobotLink* link;
		while (object_parent != NULL) {
			if (robot_name == object_parent->m_name) {
				return true;
			}
			if (!f_found_parent_link) {
				// try casting to cRobotLink
				link = dynamic_cast<cRobotLink*> (object_parent);
				if (link != NULL) {
					f_found_parent_link = true;
					ret_link_name = link->m_name;
					// position is with respect to the graphic object. need to go up to the link frame
					pos = transform * pos;
					ret_pos << pos.x(), pos.y(), pos.z();
				} else {
					transform = object_parent->getLocalTransform()*transform;
				}
			}
			object_parent = object_parent->getParent();
		}
	}
	return false;
}

// get camera object
cCamera* Sai2Graphics::getCamera(const std::string& camera_name) {
	cCamera* camera;
	// find camera among children
	// TODO: this can be optimized by locally storing list of cameras
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		cGenericObject* child = _world->getChild(i);
		if (camera_name == child->m_name) {
			camera = dynamic_cast<cCamera*>(child);
			if (camera) {
				break;
			}
		}
	}
	if (!camera) {
		cerr << "Could not find camera named " << camera_name << endl;
		abort();
		//TODO: throw exception instead
	}
	return camera;
}

cRobotLink* Sai2Graphics::findLinkObjectInParentLinkRecursive(cRobotLink* parent, const std::string& link_name) {
	// call on children
	cRobotLink* child;
	cRobotLink* ret_link = NULL;
	for (unsigned int i = 0; i < parent->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(parent->getChild(i));
		if (child != NULL) {
			if (child->m_name == link_name) {
				ret_link = child;
				break;
			} else {
				ret_link = findLinkObjectInParentLinkRecursive(child, link_name);
			}
		}
	}
	return ret_link;
}

cRobotLink* Sai2Graphics::findLink(const std::string& robot_name, const std::string& link_name) {
	// get robot base
	cRobotBase* base = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (robot_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			base = dynamic_cast<cRobotBase*>(_world->getChild(i));
			if (base != NULL) {
				break;
			}
		}
	}
	if (base == NULL) {
		//TODO: throw exception
		cerr << "Could not find robot in chai world: " << robot_name << endl;
		abort();
	}

	cRobotLink* target_link = NULL;
	// get target link
	cRobotLink* base_link;
	for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
		base_link = dynamic_cast<cRobotLink*>(base->getChild(i));
		if (base_link != NULL) {
			if (base_link->m_name == link_name) {
				target_link = base_link;
				break;
			} else {
				target_link = findLinkObjectInParentLinkRecursive(base_link, link_name);
				if (target_link != NULL) {
					break;
				}
			}
		}
	}
	return target_link;
}

void Sai2Graphics::showLinkFrameRecursive(cRobotLink* parent,
											bool show_frame,
											const double frame_pointer_length) {
	// call on children
	cRobotLink* child;
	for (unsigned int i = 0; i < parent->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(parent->getChild(i));
		if (child != NULL) {
			child->setFrameSize(frame_pointer_length, false);
			child->setShowFrame(show_frame, false);
			showLinkFrameRecursive(child, show_frame, frame_pointer_length);
		}
	}
}

// Show frame for a particular link or all links on a robot.
void Sai2Graphics::showLinkFrame(bool show_frame,
                         			const std::string& robot_name,
                         			const std::string& link_name,
                         			const double frame_pointer_length) {
	bool fShouldApplyAllLinks = false;
	if (link_name.empty()) {
		fShouldApplyAllLinks = true;
	}

	// apply frame show
	if (fShouldApplyAllLinks) {
		// get robot base
		cRobotBase* base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
			if (robot_name == _world->getChild(i)->m_name) {
				// cast to cRobotBase
				base = dynamic_cast<cRobotBase*>(_world->getChild(i));
				if (base != NULL) {
					break;
				}
			}
		}
		if (base == NULL) {
			//TODO: throw exception
			cerr << "Could not find robot in chai world: " << robot_name << endl;
			abort();
		}
		base->setWireMode(show_frame, true);
		base->setFrameSize(frame_pointer_length, false);
		base->setShowFrame(show_frame, false);
		// get target link
		cRobotLink* base_link;
		for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
			base_link = dynamic_cast<cRobotLink*>(base->getChild(i));
			if (base_link != NULL) {
				base_link->setFrameSize(frame_pointer_length, false);
				base_link->setShowFrame(show_frame, false);
				showLinkFrameRecursive(base_link, show_frame, frame_pointer_length);
			}
		}
	} else {
		auto target_link = findLink(robot_name, link_name);
		// set wireframe whenever we show frame
		target_link->setWireMode(show_frame, true);
		target_link->setFrameSize(frame_pointer_length, false);
		target_link->setShowFrame(show_frame, false);
	}
}

// Show wire mesh for a particular link or all links on a robot.
void Sai2Graphics::showWireMeshRender(bool show_wiremesh,
									const std::string& robot_name,
									const std::string& link_name) {
	bool fShouldApplyAllLinks = false;
	if (link_name.empty()) {
		fShouldApplyAllLinks = true;
	}

	// apply frame show
	if (fShouldApplyAllLinks) {
		// get robot base
		cRobotBase* base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
			if (robot_name == _world->getChild(i)->m_name) {
				// cast to cRobotBase
				base = dynamic_cast<cRobotBase*>(_world->getChild(i));
				if (base != NULL) {
					break;
				}
			}
		}
		if (base == NULL) {
			//TODO: throw exception
			cerr << "Could not find robot in chai world: " << robot_name << endl;
			abort();
		}
		base->setWireMode(show_wiremesh, true);
	} else {
		auto target_link = findLink(robot_name, link_name);

		// set wireframe whenever we show frame
		target_link->setWireMode(show_wiremesh, true);
	}
}

}
