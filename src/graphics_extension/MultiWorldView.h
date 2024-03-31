/**
 * @file MultiWorldView.h
 * @author William Chong (wmchong@stanford.edu)
 * @brief Handles rendering of views from different worlds 
 * @version 0.1
 * @date 2024-03-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MULTI_WORLD_VIEW_H
#define MULTI_WORLD_VIEW_H

#include "Sai2Graphics.h"

namespace Sai2Graphics {

class MultiWorldView {
public:
    MultiWorldView(const std::vector<std::string>& path_to_world_file,
                   const std::vector<std::string>& camera_names,
                   const std::string& window_name = "sai2 multi-world",
                   const bool verbose = false);

    // dtor
    ~MultiWorldView() {
        glfwDestroyWindow(_window);
        glfwTerminate();
        for (int i = 0; i < _graphics.size(); ++i) {
            _graphics[i]->clearWorld();
        }
    }

    void initializeWindow(const std::string& window_name);
    void renderGraphicsWorld();
    void resetWorld(const bool verbose = false);

    std::shared_ptr<Sai2Graphics> getGraphics(const int ind = 0) {
        return _graphics[ind];
    }

    bool isWindowOpen() { return !glfwWindowShouldClose(_window); }

private:
    GLFWwindow* _window;
    std::vector<std::shared_ptr<Sai2Graphics>> _graphics;
    std::vector<std::string> _camera_names;
    std::vector<std::string> _path_to_world_file;

    /**
	 * @brief used to perform side-by-side rendering 
	 * 
	 */
	chai3d::cCamera* _camera_main;
	std::vector<chai3d::cViewPanel*> _view_panels;
	std::vector<chai3d::cFrameBufferPtr> _panel_frame_buffer;
    int _window_width, _window_height;
};

}  // namespace

#endif