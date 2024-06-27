/**
 * @file test.cpp
 * @author William Chong (wmchong@stnaford.edu)
 * @brief 
 * @version 0.1
 * @date 2024-02-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "Sai2Graphics.h"
#include "redis/RedisClient.h"
#include <iostream>
#include <fstream>

const std::string fname = "image1.bin";
const string CAMERA_FRAME_KEY = "sai2::graphics::camera_frame_data";

int main() {

    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // get image data 
    auto binary_camera_data = redis_client.getBinary(CAMERA_FRAME_KEY);
    unsigned char* data = new unsigned char[binary_camera_data.size()];
    std::copy(binary_camera_data.begin(), binary_camera_data.end(), data);

    // create image to set data
    int width = 96;
    int height = 96;
    chai3d::cImage image(width, height);
    image.setData(data, 4 * width * height);
    image.setProperties(width, height, GL_RGBA, GL_UNSIGNED_BYTE);

    // save image to confirm 
    image.saveToFile("verification.png");

    return -1;
}

