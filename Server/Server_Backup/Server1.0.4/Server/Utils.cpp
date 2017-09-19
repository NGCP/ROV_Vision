/* 
 * File:   Utils.cpp
 * Author: Khanh Le
 * Created on February 9, 2017, 11:03 PM
 */

#include "Utils.h"

Utils::Utils() {
}

Utils::Utils(const Utils& orig) {
}

Mat Utils::Resize(Mat &img, int width, int height) {
    Mat newImg;
    int h = img.rows;
    int w = img.cols;
    double r;
    Size size;

    if (width == 0 && height == 0) {
        return img;
    }
    if (width == 0) {
        r = height / double(h);
        size = Size(int(w * r), height);
    }
    else {
        r = width / double(w);
        size = Size(width, int(h * r));
    }

    resize(img, newImg, size, INTER_AREA);
    return newImg;
}

Utils::~Utils() {
}

