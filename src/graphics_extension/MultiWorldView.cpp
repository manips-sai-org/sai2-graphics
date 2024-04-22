/**
 * @file MultiWorldView.cpp
 * @author William Chong (wmchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2024-03-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "MultiWorldView.h"

#include <deque>
#include <iostream>
#include <unordered_map>

#ifdef MACOSX
#include <filesystem>
#endif

#include "parser/UrdfToSai2Graphics.h"

namespace {
// custom aliases that capture the key function
#define ZOOM_IN_KEY GLFW_KEY_A
#define ZOOM_OUT_KEY GLFW_KEY_Z
#define CAMERA_RIGHT_KEY GLFW_KEY_RIGHT
#define CAMERA_LEFT_KEY GLFW_KEY_LEFT
#define CAMERA_UP_KEY GLFW_KEY_UP
#define CAMERA_DOWN_KEY GLFW_KEY_DOWN
#define NEXT_CAMERA_KEY GLFW_KEY_N
#define PREV_CAMERA_KEY GLFW_KEY_B
#define SHOW_CAMERA_POS_KEY GLFW_KEY_S

// map to store key presses. The first bool is true if the key is pressed and
// false otherwise, the second bool is used as a flag to know if the initial key
// press has been consumed when something needs to happen only once when the key
// is pressed and not continuously
std::unordered_map<int, std::pair<bool, bool>> key_presses_map = {
	{ZOOM_IN_KEY, std::make_pair(false, true)},
	{ZOOM_OUT_KEY, std::make_pair(false, true)},
	{CAMERA_RIGHT_KEY, std::make_pair(false, true)},
	{CAMERA_LEFT_KEY, std::make_pair(false, true)},
	{CAMERA_UP_KEY, std::make_pair(false, true)},
	{CAMERA_DOWN_KEY, std::make_pair(false, true)},
	{NEXT_CAMERA_KEY, std::make_pair(false, true)},
	{PREV_CAMERA_KEY, std::make_pair(false, true)},
	{SHOW_CAMERA_POS_KEY, std::make_pair(false, true)},
	{GLFW_KEY_LEFT_SHIFT, std::make_pair(false, true)},
	{GLFW_KEY_LEFT_ALT, std::make_pair(false, true)},
	{GLFW_KEY_LEFT_CONTROL, std::make_pair(false, true)},
};

std::unordered_map<int, std::pair<bool, bool>> mouse_button_presses_map = {
	{GLFW_MOUSE_BUTTON_LEFT, std::make_pair(false, true)},
	{GLFW_MOUSE_BUTTON_RIGHT, std::make_pair(false, true)},
	{GLFW_MOUSE_BUTTON_MIDDLE, std::make_pair(false, true)},
};

std::deque<double> mouse_scroll_buffer;

bool is_pressed(int key) {
	if (key_presses_map.count(key) > 0) {
		return key_presses_map.at(key).first;
	}
	if (mouse_button_presses_map.count(key) > 0) {
		return mouse_button_presses_map.at(key).first;
	}
	return false;
}

bool consume_first_press(int key) {
	if (key_presses_map.count(key) > 0) {
		if (key_presses_map.at(key).first && key_presses_map.at(key).second) {
			key_presses_map.at(key).second = false;
			return true;
		}
	}
	if (mouse_button_presses_map.count(key) > 0) {
		if (mouse_button_presses_map.at(key).first &&
			mouse_button_presses_map.at(key).second) {
			mouse_button_presses_map.at(key).second = false;
			return true;
		}
	}
	return false;
}

// callback to print glfw errors
void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action,
			   int mods) {
	bool set = (action != GLFW_RELEASE);
	if (key == GLFW_KEY_ESCAPE) {
		// handle esc separately to exit application
		glfwSetWindowShouldClose(window, GL_TRUE);
	} else {
		if (key_presses_map.count(key) > 0) {
			key_presses_map.at(key).first = set;
			if (!set) {
				key_presses_map.at(key).second = true;
			}
		}
	}
}

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	if (mouse_button_presses_map.count(button) > 0) {
		mouse_button_presses_map.at(button).first = set;
		if (!set) {
			mouse_button_presses_map.at(button).second = true;
		}
	}
}

// callback when the mouse wheel is scrolled
void mouseScroll(GLFWwindow* window, double xoffset, double yoffset) {
	if (yoffset != 0) {
		mouse_scroll_buffer.push_back(yoffset);
	} else if (xoffset != 0) {
		mouse_scroll_buffer.push_back(xoffset);
	}
}

GLFWwindow* glfwInitialize(const std::string& window_name, const int n_window) {
	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
    // int windowW = 0.9 * screenW;
	// int windowH = 0.9 * screenH;
	// int windowPosY = (screenH - windowH) / 2;
	// int windowPosX = windowPosY;

	// create window and make it current context
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window =
	glfwCreateWindow(windowW, windowH, window_name.c_str(), NULL, NULL);
	if (n_window == 0) {
		glfwSetWindowPos(window, 0, screenH/4);
	} else {
		glfwSetWindowPos(window, screenW/2, screenH/4);
	}
	// glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}
}  // namespace

namespace Sai2Graphics {

MultiWorldView::MultiWorldView(const std::vector<std::string>& path_to_world_file,
                               const std::vector<std::string>& camera_names,
							   const bool side_by_side,
                               const std::string& window_name,
                               const bool verbose) : _path_to_world_file(path_to_world_file), 
                               _camera_names(camera_names), _side_by_side(side_by_side)
{
    for (auto path : path_to_world_file) {
        _graphics.push_back(std::make_shared<Sai2Graphics>(path, window_name, verbose, true));
    }

    // setup view panels and attach to one camera from each world (supports only 2 for now)
    _camera_main = new chai3d::cCamera(NULL);
    _camera_primary = new chai3d::cCamera(NULL);
    _camera_secondary = new chai3d::cCamera(NULL);
    for (int i = 0; i < 2; ++i) {
        _panel_frame_buffer.push_back(chai3d::cFrameBuffer::create());
        // todo: check camera names are valid
        _panel_frame_buffer[i]->setup(_graphics[i]->getCamera(_camera_names[i]));
        _view_panels.push_back(new chai3d::cViewPanel(_panel_frame_buffer[i]));
		if (_side_by_side) {
        	_camera_main->m_frontLayer->addChild(_view_panels[i]);
		} else {
			if (i == 0) {        
				_camera_primary->m_frontLayer->addChild(_view_panels[i]);
			} else {
				_camera_secondary->m_frontLayer->addChild(_view_panels[i]);
			}
		}
    }

	if (!_side_by_side) {
		for (int i = 0; i < 2; ++i) {
			#ifdef MACOSX
				auto path = std::__fs::filesystem::current_path();
				initializeWindow(window_name, i);
				std::__fs::filesystem::current_path(path);
			#else
				initializeWindow(window_name, i);
			#endif
			_graphics[i]->setWindow(_window[i]);
		}
	} else {
		#ifdef MACOSX
			auto path = std::__fs::filesystem::current_path();
			initializeWindow(window_name);
			std::__fs::filesystem::current_path(path);
		#else
			initializeWindow(window_name);
		#endif
	}

}

void MultiWorldView::initializeWindow(const std::string& window_name, const int n_window) {
	_window.push_back(glfwInitialize(window_name, n_window));

	// set callbacks
	glfwSetKeyCallback(_window[_window.size() - 1], keySelect);
	glfwSetMouseButtonCallback(_window[_window.size() - 1], mouseClick);
	glfwSetScrollCallback(_window[_window.size() - 1], mouseScroll);
}

void MultiWorldView::resetWorld(const bool verbose) {
    for (int i = 0; i < _graphics.size(); ++i) {
        _graphics[i]->resetWorld(_path_to_world_file[i], verbose);
    }

    // clear previous setup
    _camera_main = NULL;
	_camera_primary = NULL; 
	_camera_secondary = NULL; 
    _panel_frame_buffer.clear();
    _view_panels.clear();

    // setup frame buffers
    _camera_main = new chai3d::cCamera(NULL);
	_camera_primary = new chai3d::cCamera(NULL);
	_camera_secondary = new chai3d::cCamera(NULL);
    for (int i = 0; i < 2; ++i) {
        _panel_frame_buffer.push_back(chai3d::cFrameBuffer::create());
        // todo: check camera names are valid
        _panel_frame_buffer[i]->setup(_graphics[i]->getCamera(_camera_names[i]));
        _view_panels.push_back(new chai3d::cViewPanel(_panel_frame_buffer[i]));
		if (_side_by_side) {
        	_camera_main->m_frontLayer->addChild(_view_panels[i]);
		} else {
			if (i == 0) {
				_camera_primary->m_frontLayer->addChild(_view_panels[i]);
			} else {
				_camera_secondary->m_frontLayer->addChild(_view_panels[i]);
			}
		}
    }
}

void MultiWorldView::renderGraphicsWorld() {

	if (!_side_by_side) {
		for (int i = 0; i < 2; ++i) {
			glfwMakeContextCurrent(_window[i]);
			_graphics[i]->renderGraphicsWorld(_camera_names[i]);
		}
	} else {
		// setup frame buffer window sizes (supports only 2 at this point)
		int halfW = _window_width / 2;
		int halfH = _window_height / 2;
		int offset = 1;

		// update display panel sizes and positions
		_view_panels[0]->setLocalPos(0.0, 0.0);
		_view_panels[0]->setSize(halfW, _window_height);

		_view_panels[1]->setLocalPos(halfW+offset, 0.0);
		_view_panels[1]->setSize(halfW, _window_height);

		// update frame buffer sizes
		_panel_frame_buffer[0]->setSize(halfW, _window_height);
		_panel_frame_buffer[1]->setSize(halfW, _window_height);

		// render frame buffers
		_panel_frame_buffer[0]->renderView();
		_panel_frame_buffer[1]->renderView();

		_camera_main->renderView(_window_width, _window_height);
	}
}

}  // namespace 