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
#include "chai_extension/CRobotLink.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

namespace Sai2Graphics {

class Sai2Graphics {
public:
	/**
     * @brief Creates a Chai graphics interface object that contains a visual model of the virtual world.
     * @param path_to_world_file A path to the file containing the model of the virtual world (urdf and yml files supported).
     * @param verbose To display information about the robot model creation in the terminal or not.
     */
	Sai2Graphics(const std::string& path_to_world_file,
               const std::string& window_name = "sai2 world",
               bool verbose = false);

	// dtor
	~Sai2Graphics();

     /**
      * @brief resets the rendered world and re initializes it with the new world file
      * 
      * @param path_to_world_file world file to render
      * @param verbose print info to terminal or not
      */
     void resetWorld(const std::string& path_to_world_file, const bool verbose = false);

     /**
      * @brief returns true is the window is open and should stay open
      * 
      * @return true 
      * @return false 
      */
     bool isWindowOpen() {
          return !glfwWindowShouldClose(_window);
     }

     /**
      * @brief updates the window display with the most current information
      * needs to ba called after updateGraphics, updateObjectGraphics
      * (and render for now) if the changes of robot or object positions are to be
      * displayed
      * 
      * @param camera_name the name of the camera to display
      */
     void updateDisplayedWorld(const std::string &camera_name);

	/**
     * @brief Update the graphics model for a robot in the virtual world.
     * @param robot_name Name of the robot for which model update is considered.
     * @param robot_model A ModelInterface object for this robot to be used to get intermediate transforms.
     */
	void updateGraphics(const std::string& robot_name,
						Sai2Model::Sai2Model* robot_model);

     /**
     * @brief Update the graphics model for an object in the virtual world.
     * @param object_name Name of the object for which model update is considered.
     * @param object_pos  position of the object in the world
     * @param object_ori  orientation of the object in the world
     */
     void updateObjectGraphics(const std::string& object_name,
                         const Eigen::Vector3d& object_pos,
                         const Eigen::Quaterniond object_ori);

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
     void showLinkFrame(bool show_frame,
                         const std::string& robot_name,
                         const std::string& link_name = "",
                         const double frame_pointer_length = 0.03);

     /**
     * @brief Render wire mesh for a particular link or all links on a robot.
     * @param show_wiremesh Flag whether should show wire mesh or not.
     * @param robot_name Robot name.
     * @param robot_name Link name. If left blank, all link frames are shown.
     */
     void showWireMeshRender(bool show_wiremesh,
                         const std::string& robot_name,
                         const std::string& link_name = "");

private:
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
     * @param display_context_id ID for the context to display in. This ID is only to be used
     *	for selective rendering. It does not change the GL context to render to.
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

     /**
     * @brief Get info about link of the specified robot at the given cursor position.
     * @return True if a link is present, False if not
     * @param camera_name Camera name.
     * @param robot_name Name of robot to look for.
     * @param view_x x-position of cursor in viewport (OpenGL style screen co-ordinates).
     * @param view_y y-position of cursor in viewport (OpenGL style screen co-ordinates).
     * @param window_width Width of viewport in screen co-ordinates.
     * @param window_height Height of viewport in screen co-ordinates.
     * @param ret_link_name Name of the link. Garbage if no link present at cursor location.
     * @param ret_pos Position of cursor in link frame. Garbage if no link present at cursor location.
     */
     bool getRobotLinkInCamera(const std::string& camera_name,
                                   const std::string& robot_name,
                                   int view_x,
                                   int view_y,
                                   int window_width,
                                   int window_height,
                                   std::string& ret_link_name,
                                   Eigen::Vector3d& ret_pos);

     /* CHAI specific interface */
     /**
     * @brief Get pointer to Chai camera object.
     * @param camera_name Camera name.
     */
     chai3d::cCamera* getCamera(const std::string& camera_name);

     /**
     * internal functions to find link
     */
     chai3d::cRobotLink* findLinkObjectInParentLinkRecursive(chai3d::cRobotLink* parent, const std::string& link_name);

     chai3d::cRobotLink* findLink(const std::string& robot_name, const std::string& link_name);

     void showLinkFrameRecursive(chai3d::cRobotLink* parent, bool show_frame, const double frame_pointer_length);


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
      * @brief position and direction of view for current camera
      * 
      */

     /**
      * @brief position of the camera and lookat point
      * 
      */
	Vector3d _camera_pos;
     Vector3d _camera_lookat_point;
     Vector3d _camera_up_axis;

     /**
      * @brief used to store the last cursor position
      * useful when interacting with the window to move the camera
      * 
      */
	double _last_cursorx;
     double _last_cursory;

     int _window_width;
     int _window_height;

};

} // namespace

#endif //CHAI_GRAPHICS_H
