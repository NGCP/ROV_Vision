#include "ObjectDetection.h"

using namespace std;

Scalar colorTable[NUM_COLOR][2] = {
    {Scalar(20, 45, 19), Scalar(75 , 255, 255)}, // Yellow - 88 for normal yellow, 44 for yellow in the pool
    {Scalar(16, 86, 27), Scalar(72, 255, 255)}, // Green
    {Scalar(138, 154, 47), Scalar(255, 255, 215)}, // Red
};

bool ShapeDetection(Mat &img, Mat &mask) {
    Contour contour(mask);
    contour.FindContourArea();
    contour.SortContourByArea();
    bool isDetected = contour.DrawContour(img);
    return isDetected;
}

void Preprocessing(Mat &img, Mat &mask) {
    Mat blurredImg;
    GaussianBlur(img, blurredImg, Size(15, 15), 0, 0);
    Mat hsvImg;
    cvtColor(blurredImg, hsvImg, CV_BGR2HSV);
    Mat rangeMask;
    inRange(hsvImg, colorTable[YELLOW][LOWER], colorTable[YELLOW][UPPER], rangeMask);

    Mat grayMask;
    cvtColor(blurredImg, grayMask, CV_BGR2GRAY);
    adaptiveThreshold(grayMask, grayMask, 255,
            ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 4);
    rangeMask = grayMask.mul(rangeMask);

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
    Mat erodeMask, dilateMask;
    erode(rangeMask, erodeMask, element);
    erode(erodeMask, erodeMask, element);
    dilate(erodeMask, dilateMask, element);
    dilate(dilateMask, dilateMask, element);
    mask = dilateMask;
}

bool RunObjectDetection(Mat &img) {
    Mat mask;
    Preprocessing(img, mask);
    bool isDetected = ShapeDetection(img, mask);
    return isDetected;
}
