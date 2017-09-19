#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

#include "Library.h"
#include "Utils.h"
#include "Contour.h"
#include "Noise.h"

bool ShapeDetection(Mat &img, Mat &mask);
void Preprocessing(Mat &img, Mat &mask);
bool RunObjectDetection(Mat &img);

#endif
