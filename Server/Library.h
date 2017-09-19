/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Library.h
 * Author: Khanh Le
 *
 * Created on February 9, 2017, 11:18 PM
 */

#ifndef LIBRARY_H
#define LIBRARY_H

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp> // for video
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include <time.h>
#define LOWER 0
#define UPPER 1

using namespace cv;

typedef enum {
    YELLOW = 0,
    GREEN = 1,
    RED = 2,
    NUM_COLOR = 3
} Color;

#endif /* LIBRARY_H */

