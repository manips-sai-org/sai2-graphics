/**
 * \file Sai2Graphics.h
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#ifndef SAI2_GRAPHICS_H
#define SAI2_GRAPHICS_H

#include <chai3d.h>

#include "Sai2Model.h"
#include "widgets/ForceSensorDisplay.h"
#include "widgets/UIForceWidget.h"

// clang-format off
#include <GLFW/glfw3.h>	 //must be loaded after loading opengl/glew
// clang-format on

#include <unordered_map>

namespace Sai2Graphics {

class Sai2Graphics {
public:
	/**
	 * @brief Creates a Chai graphics interface object that contains a visual
	 * model of the virtual world.
	 * @param path_to_world_file A path to the file containing the model of the
	 * virtual world (urdf and yml files supported).
	 * @param verbose To display information about the robot model creation in
	 * the terminal or not.
	 * @param headless To create a display window or not (true if using MultiWorldView)
	 */
	Sai2Graphics(const std::string& path_to_world_file,
				 const std::string& window_name = "sai2 world",
				 bool verbose = false,
				 bool headless = false);

	// dtor
	~Sai2Graphics();

	/**
	 * @brief resets the rendered world and re initializes it with the new world
	 * file
	 *
	 * @param path_to_world_file world file to render
	 * @param verbose print info to terminal or not
	 */
	void resetWorld(const std::string& path_to_world_file,
					const bool verbose = false);

	/**
	 * @brief returns true is the window is open and should stay open
	 *
	 * @return true
	 * @return false
	 */
	bool isWindowOpen() { return !glfwWindowShouldClose(_window); }

	/**
	 * @brief renders the graphics world from the current camera (needs to be
	 * called after all the update functions i.e. updateRobotGraphics)
	 *
	 * @param camera_name the name of the camera to display
	 */
	void renderGraphicsWorld(const std::string& render_camera_name = "");

	/**
	 * @brief sets the glfw window pointer 
	 * 
	 * @param window 
	 */
	void setWindow(GLFWwindow* window) {
		_window = window;
	}

	/**
	 * @brief remove all interactions widgets
	 * after calling that function, right clicking on the window won't
	 * display lines and generate forces/joint torques
	 *
	 */
	void clearUIForceWidgets() { _ui_force_widgets.clear(); }

	/**
	 * @brief get the joint torques from the ui interaction (right click on
	 * robot link) for a given robot
	 *
	 * @param robot_name name of the robot for which we want the joint torques
	 * @return joint torques from UI interaction for that robot
	 */
	Eigen::VectorXd getUITorques(const std::string& robot_name);

	/**
	 * @brief Enable interacting with a specific robot by right clicking on the
	 * display window
	 *
	 * @param robot_name name of the robot
	 * @param robot_model model of the robot
	 */
	void addUIForceInteraction(const std::string& robot_name,
							   const bool interact_at_object_center = false);

	/**
	 * @brief Get the the names of the robots in the graphics world
	 *
	 * @return vector of robot names
	 */
	const std::vector<std::string> getRobotNames() const;

	/**
	 * @brief Get the the names of the objects in the graphics world
	 *
	 * @return vector of object names
	 */
	const std::vector<std::string> getObjectNames() const;

	/**
	 * @brief Update the graphics model for a robot in the virtual world.
	 * @param robot_name Name of the robot for which model update is considered.
	 * @param joint_angles joint angles for that robot
	 */
	void updateRobotGraphics(const std::string& robot_name,
							 const Eigen::VectorXd& joint_angles,
							 const Eigen::VectorXd& joint_velocities);
	void updateRobotGraphics(const std::string& robot_name,
							 const Eigen::VectorXd& joint_angles);

	/**
	 * @brief Update the graphics model for an object in the virtual world.
	 * @param object_name Name of the object for which model update is
	 * considered.
	 * @param object_pose  pose of the object in the world
	 */
	void updateObjectGraphics(
		const std::string& object_name, const Eigen::Affine3d& object_pose,
		const Eigen::Vector6d& object_velocity = Eigen::Vector6d::Zero());

	void showObjectLinkFrame(bool show_frame, const std::string& object_name,
							 const double frame_pointer_length = 0.03);

	Eigen::VectorXd getRobotJointPos(const std::string& robot_name);

	Eigen::Affine3d getObjectPose(const std::string& object_name);

	/**
	* @brief Show frame for a particular link or all links on a robot.
		 This also causes the link graphics as well as graphics for any
		 child link to be displayed as wire mesh to allow the frame to be
		 seen.
	* @param show_frame Flag whether should show frame or not.
	* @param robot_name Robot name.
	* @param robot_name Link name. If left blank, all link frames are shown.
	* @param frame_pointer_length Axis arrow length in meters.
	*/
	void showLinkFrame(bool show_frame, const std::string& robot_name,
					   const std::string& link_name = "",
					   const double frame_pointer_length = 0.20);

	/**
	 * @brief Render wire mesh for a particular link or all links on a robot.
	 * @param show_wiremesh Flag whether should show wire mesh or not.
	 * @param robot_name Robot name.
	 * @param robot_name Link name. If left blank, all link frames are shown.
	 */
	void showWireMesh(bool show_wiremesh, const std::string& robot_name,
					  const std::string& link_name = "");

	void showTransparency(bool show_transparency, const std::string& robot_name, const double level);

	void showObjectTransparency(bool show_transparency, const std::string& object_name, const double level);

	void setBackgroundColor(const double red, const double green,
							const double blue) {
		_world->setBackgroundColor(red, green, blue);
	}

	std::string getCurrentCameraName() const {
		return _camera_names[_current_camera_index];
	}

	void addForceSensorDisplay(const Sai2Model::ForceSensorData& sensor_data);

	void updateDisplayedForceSensor(
		const Sai2Model::ForceSensorData& force_data);

	bool isKeyPressed(int key) const {
		return glfwGetKey(_window, key) == GLFW_PRESS;
	}

	void setRenderingEnabled(const bool rendering_enabled,
							 const string robot_or_object_name,
							 const string link_name = "");

	bool modelExistsInWorld(const std::string& model_name) const
	{
		return robotExistsInWorld(model_name) || objectExistsInWorld(model_name);
	}

	bool robotExistsInWorld(const std::string& robot_name,
									const std::string& link_name = "") const;

	bool objectExistsInWorld(const std::string& object_name) const;

	// functions related to frame buffer loading and saving
	void addFrameBuffer(const std::string camera_name,
						const int width = 1280, const int height = 720);
	void saveFrameBuffer(const std::string camera_name, 
						 const std::string fname);
	void writeFrameBuffer(const std::string camera_name,
						  const std::string fname);
	std::vector<unsigned char> getFrameBuffer(const std::string camera_name);

	// camera assignment to follow a robot link
	void setCameraOnRobot(const std::string& camera_name,
						  const std::string& robot_name,
						  const std::string& link_name,
						  const Eigen::Affine3d& rel_pose,
						  const std::vector<int>& order = {2, 1});  // look at, up

	void setCameraFov(const std::string& camera_name,
					  const double& fov_rad);

	/**
	 * @brief Get pointer to Chai camera object.
	 * @param camera_name Camera name.
	 */
	chai3d::cCamera* getCamera(const std::string& camera_name);

	void clearWorld();

	int getWindowWidth() { return _window_width; }
	int getWindowHeight() { return _window_height; }

	void duplicateRobot(const std::string& robot_name,
						const int n_copies) {
		
	}

	/**
	 * @brief Add label
	 * 
	 * @param label_name label name to refer to in map
	 * @param camera_name camera to render text to
	 */
	void addLabel(const std::string& label_name,
				  const std::string& camera_name) {
		auto font = chai3d::NEW_CFONTCALIBRI20();  // check https://www.chai3d.org/download/doc/html/namespacechai3d.html#af8ad087918683eb3afe1669c641f0516 for fonts
		_labels[label_name] = new chai3d::cLabel(font);
		_labels[label_name]->m_fontColor.setBlack();  
		getCamera(camera_name)->m_frontLayer->addChild(_labels[label_name]);
	}

	/**
	 * @brief Update label
	 * 
	 * @param label_name label name to refer to in map
	 * @param text text to display 
	 * @param x_location x pixel location
	 * @param y_location y pixel location 
	 */
	void updateLabel(const std::string& label_name,
					 const std::string& text,
				     const int& x_location,
					 const int& y_location = 15) {
		_labels[label_name]->setText(text);
		_labels[label_name]->setLocalPos(x_location, y_location);
		// m_label[name]->->setLocalPos((int)(0.5 * (width - m_label[name]->->getWidth())), y_location);
	}

	/**
	 * @brief Add spheres to the world 
	 * 
	 */
	void addSphere(const Vector3d& origin, 
				   const double radius,
				   const int color_opt = 0) {
		auto sphere = new chai3d::cShapeSphere(radius);
		auto material = std::make_shared<chai3d::cMaterial>();
		if (color_opt == 0) {
			material->setGreenForest();
		} else {
			material->setRed();
		}
		sphere->setMaterial(material);
		sphere->setLocalPos(chai3d::cVector3d(origin));
		_world->addChild(sphere);
	}

	/**
	 * @brief Add block to the world 
	 * 
	 * @param origin 
	 * @param length 
	 * @param width 
	 * @param height 
	 */
	void addBlock(const Vector3d& origin,
				  const double& length,
				  const double& width,
				  const double& height) {
		auto box = new chai3d::cShapeBox(length, width, height);
		auto material = std::make_shared<chai3d::cMaterial>();
		material->setBlue();
		box->setMaterial(material);
		box->setLocalPos(chai3d::cVector3d(origin));
		_world->addChild(box);
	}

private:
	void initializeWorld(const std::string& path_to_world_file,
						 const bool verbose);
	// void clearWorld();

	/**
	 * @brief initialize the glfw window with the given window name
	 *
	 * @param window_name
	 */
	void initializeWindow(const std::string& window_name);

	/**
	 * @brief Render the virtual world to the current context.
	 * 	NOTE: the correct context should have been selected prior to this.
	 * @param camera_name Camera name to be rendered from.
	 * @param window_width Width of viewport in screen co-ordinates.
	 * @param window_height Height of viewport in screen co-ordinates.
	 * @param display_context_id ID for the context to display in. This ID is
	 *only to be used for selective rendering. It does not change the GL context
	 *to render to.
	 */
	void render(const std::string& camera_name);

	/**
	 * @brief Return the pose of the camera in the parent frame
	 * @param camera_name Camera name.
	 * @param ret_position Position of the camera.
	 * @param ret_vertical Up vector for the camera.
	 * @param ret_lookat Point the camera is looking at.
	 */
	void getCameraPose(const std::string& camera_name,
					   Eigen::Vector3d& ret_position,
					   Eigen::Vector3d& ret_vertical,
					   Eigen::Vector3d& ret_lookat);

	/**
	 * @brief Sets the pose of the camera in the parent frame
	 * @param camera_name Camera name.
	 * @param position Position of the camera.
	 * @param vertical Up vector for the camera.
	 * @param lookat Point the camera is to look at.
	 */
	void setCameraPose(const std::string& camera_name,
					   const Eigen::Vector3d& position,
					   const Eigen::Vector3d& vertical,
					   const Eigen::Vector3d& lookat);

	/* CHAI specific interface */
	// /**
	//  * @brief Get pointer to Chai camera object.
	//  * @param camera_name Camera name.
	//  */
	// chai3d::cCamera* getCamera(const std::string& camera_name);

	/**
	 * internal functions to find link
	 */
	chai3d::cRobotLink* findLinkObjectInParentLinkRecursive(
		chai3d::cRobotLink* parent, const std::string& link_name);

	chai3d::cRobotLink* findLink(const std::string& robot_name,
								 const std::string& link_name);

	void showLinkFrameRecursive(chai3d::cRobotLink* parent, bool show_frame,
								const double frame_pointer_length);

	int findForceSensorDisplay(const std::string& robot_or_object_name,
							   const std::string& link_name) const;

	/**
	 * @brief Internal cWorld object.
	 */
	chai3d::cWorld* _world;

	/**
	 * @brief glfw window
	 *
	 */
	GLFWwindow* _window;

	/**
	 * @brief the widgets responsible for handling the computation of joint
	 * torques when right clicking and dragging the mouse on the display window
	 *
	 */
	std::vector<std::shared_ptr<UIForceWidget>> _ui_force_widgets;

	bool _right_click_interaction_occurring;

	/**
	 * @brief maps from robot names to filename and from robot names to robot
	 * models
	 *
	 */
	std::map<std::string, std::string> _robot_filenames;
	std::map<std::string, std::shared_ptr<Sai2Model::Sai2Model>> _robot_models;

	std::map<std::string, std::shared_ptr<Eigen::Affine3d>> _object_poses;
	std::map<std::string, std::shared_ptr<Eigen::Vector6d>> _object_velocities;

	/**
	 * @brief force sensor displays
	 *
	 */
	std::vector<std::shared_ptr<ForceSensorDisplay>> _force_sensor_displays;

	/**
	 * @brief vector of camera names in the world and current camera index
	 *
	 */
	std::vector<std::string> _camera_names;
	int _current_camera_index;

	/**
	 * @brief used to store the last cursor position
	 * useful when interacting with the window to move the camera
	 *
	 */
	double _last_cursorx;
	double _last_cursory;

	int _window_width;
	int _window_height;

	/**
	 * @brief used to store frame buffers for recording
	 * 
	 */
	std::map<string, chai3d::cFrameBufferPtr> _frame_buffer;

	std::map<std::string, chai3d::cLabel*> _labels;

};

}  // namespace Sai2Graphics

#endif	// CHAI_GRAPHICS_H
