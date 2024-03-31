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
#include <iostream>
#include <fstream>
#include <cstdio>

const std::string fname = "image1.bin";

int main() {

    std::remove("verification.png");

    // create image to process individual frames
    // int width = 1280;
    // int height = 720;
    int width = 96;
    int height = 96;
    chai3d::cImage image(width, height);

    std::FILE* fp = std::fopen(fname.c_str(), "rb");

    int buffer_size = width * height * 4;
    unsigned char* buffer = new unsigned char[buffer_size];
    size_t bytesRead = std::fread(buffer, 4, buffer_size, fp);

    image.setData(buffer, 4 * width * height);
    image.setProperties(width, height, GL_RGBA, GL_UNSIGNED_BYTE);

    // save image to confirm 
    image.saveToFile("verification.png");

    return -1;
}