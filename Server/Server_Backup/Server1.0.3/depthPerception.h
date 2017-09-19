#ifndef __DEPTHPERCEPTION_H__
#define __DEPTHPERCEPTION_H__

#include "Library.h"

void RunDepthPerception(
   cv::Ptr<cv::StereoSGBM> stereo,
   cv::Mat img1,
   cv::Mat img2,
   cv::Mat &M1, 
   cv::Mat &M2, 
   cv::Mat &D1, 
   cv::Mat &D2,
   cv::Mat &R, 
   cv::Mat &T,
   cv::Mat &E, 
   cv::Mat &F, 
   cv::Mat &R1, 
   cv::Mat &R2,
   cv::Mat &P1,
   cv::Mat &P2,
   cv::Mat &map11,
   cv::Mat &map12,
   cv::Mat &map21,
   cv::Mat &map22,
   cv::Mat &Q 
);

cv::Ptr<cv::StereoSGBM> initializeDepthPerception(
       const char *inputXML,
      cv::Mat &M1, 
      cv::Mat &M2, 
      cv::Mat &D1, 
      cv::Mat &D2,
      cv::Mat &R, 
      cv::Mat &T,
      cv::Mat &E, 
      cv::Mat &F, 
      cv::Mat &R1, 
      cv::Mat &R2,
      cv::Mat &P1,
      cv::Mat &P2,
      cv::Mat &map11,
      cv::Mat &map12,
      cv::Mat &map21,
      cv::Mat &map22,
      cv::Mat &Q);

#endif