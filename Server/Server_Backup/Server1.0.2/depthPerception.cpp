#pragma warning( disable: 4996)
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "depthPerception.h"
using namespace std;

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
) {
	int	avgDisp = 0, total, nonZero;
	
	cv::Mat blur1, blur2, lap1, lap2;
	cv::Mat blur1a, blur2a, blur1b, blur2b, blur1c, blur2c, blur1d, blur2d;
	cv::Mat morphA, morph;
	
	pair<string, int> DistValues[16] = {
	{"< 2ft", 254},
	{"< 2ft", 250},
	{"3ft", 243},
	{"4ft", 215},
	{"5ft", 195},
	{"6ft", 179},
	{"7ft", 172},
	{"8ft", 167},
	{"9ft", 163},
	{"10ft", 154},
	{"11ft", 149},
	{"12ft", 146},
	{"13ft", 142},
	{"14ft", 139},
	{"15ft", 136},
	{"> 15ft", 0} };
	
	string outString = DistValues[15].first;

	// Actual Calculations
	cv::Mat img1r, img2r, disp, vdisp, result;

	if (img1.empty() || img2.empty())
		return;

	/*
	Applies a generic geometrical transformation to an image.
	*/
	cout << "Remapping images... ";
	cv::remap(
		img1, // Source image. 
		img1r, // Destination image. It has the same size as map1 and the same type as src .
		map11, // The first map of either (x,y) points or just x values having the type CV_16SC2 , CV_32FC1, or CV_32FC2. See convertMaps for details on converting a floating point representation to fixed-point for speed.
		map12, // The second map of y values having the type CV_16UC1, CV_32FC1, or none (empty map if map1 is (x,y) points), respectively.
		cv::INTER_LINEAR //Interpolation method (see cv::InterpolationFlags). The method INTER_AREA is not supported by this function.
							 // Pixel extrapolation method (see cv::BorderTypes). When borderMode=BORDER_TRANSPARENT, it means that the pixels in the destination image that corresponds to the "outliers" in the source image are not modified by the function.
							 // Value used in case of a constant border. By default, it is 0.
	);
	cv::remap(
		img2,
		img2r,
		map21,
		map22,
		cv::INTER_LINEAR
	);
		
	//cv::imshow("Remapped Left Image", img1r);
	//cv::imshow("Remapped Right Image", img2r);
	//if ((cv::waitKey() & 255) == 27)
	//	break;
	medianBlur(img1r, blur1a, 15);
	medianBlur(img2r, blur2a, 15);

	medianBlur(blur1a, blur1b, 15);
	medianBlur(blur2a, blur2b, 15);

	medianBlur(blur1b, blur1c, 15);
	medianBlur(blur2b, blur2c, 15);

	medianBlur(blur1c, blur1d, 15);
	medianBlur(blur2c, blur2d, 15);

	medianBlur(blur1d, blur1, 15);
	medianBlur(blur2d, blur2, 15);

	/*
	Computes disparity map for the specified stereo pair.
	*/
		
	stereo->compute(
		blur1, // Left 8-bit single-channel image.
		blur2, // Right image of the same size and the same type as the left one.
		disp // Output disparity map. It has the same size as the input images. Some algorithms, like StereoBM or StereoSGBM compute 16-bit fixed-point disparity map (where each disparity value has 4 fractional bits), whereas other algorithms output 32-bit floating-point disparity map.
	);

	// cv::imshow("Pre-normalization Disparity Map", disp);
	//if ((cv::waitKey() & 255) == 27)
	//	break;

	cv::normalize(
		disp, // input array.
		vdisp, // output array of the same size as src .
		0, // norm value to normalize to or the lower range boundary in case of the range normalization.
		256, // upper range boundary in case of the range normalization; it is not used for the norm normalization.
		cv::NORM_MINMAX, // normalization type (see cv::NormTypes).
		CV_8U // when negative, the output array has the same type as src; otherwise, it has the same number of channels as src and the depth =CV_MAT_DEPTH(dtype).
			  // optional operation mask.
	);
	
	//cv::imshow("Normalized Disparity", vdisp);
	//if ((cv::waitKey() & 255) == 27)
	//	break;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
	cv::dilate(vdisp, morphA, element, cv::Point(-1, -1), 50, cv::BORDER_DEFAULT, cv::morphologyDefaultBorderValue());
	cv::erode(morphA, morph, element, cv::Point(-1, -1), 40, cv::BORDER_DEFAULT, cv::morphologyDefaultBorderValue());


	//Then define your mask image
	cv::Mat mask = cv::Mat::zeros(vdisp.size(), vdisp.type());

	//Define your destination image
	cv::Mat dstImage = cv::Mat::zeros(vdisp.size(), vdisp.type());
	//Draw rectangle mask in center
	cv::rectangle(mask, cv::Point((mask.cols / 2) - 25, (mask.rows / 2) + 25), cv::Point((mask.cols / 2) + 25, (mask.rows / 2) - 25), cv::Scalar(255), -1, cv::LINE_8, 0);
		
	//cv::imshow("MASK", mask);
	//if ((cv::waitKey() & 255) == 27)
	//	break;

	//Now you can copy your source image to destination image with masking
	morph.copyTo(dstImage, mask);

	cv::Scalar sum = cv::sum(dstImage);
	total = sum[0];
	nonZero = cv::countNonZero(dstImage);

	//cv::putText(dstImage, to_string(nonZero), cv::Point2f(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255), 1, 4, false);
	//cv::imshow("Morphed", morph);
	//if ((cv::waitKey() & 255) == 27)
	//	break;

	if (nonZero)
		avgDisp = total / nonZero;
	
	for (int i = 15; i >= 0; i--) {
			if (avgDisp > DistValues[i].second) {
				outString = DistValues[i].first;
			}
			else
				break;
	}
	

	//cout << "Middle pixel val: " << to_string(avgDisp);
	//cv::rectangle(morph, cv::Point(295, 265), cv::Point(345, 215), cv::Scalar(255), 3, cv::LINE_8, 0);
	//cv::putText(morph, outString, cv::Point2f(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255), 1, 4, false);
	//cv::rectangle(img1, cv::Point(295, 265), cv::Point(345, 215), cv::Scalar(255), 3, cv::LINE_8, 0);
	cv::circle(img1, cv::Point(320, 240), 25, cv::Scalar(0,0,255), 2, 8, 0);
	cv::line(img1, cv::Point(303, 240), cv::Point(337, 240), cv::Scalar(0,0,255), 1, 8, 0);
	cv::line(img1, cv::Point(320, 257), cv::Point(320, 223), cv::Scalar(0,0,255), 1, 8, 0);
    cv::putText(img1, outString, cv::Point2f(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0,0,255), 1, 4, false);

	//cv::rectangle(img2, cv::Point(295, 265), cv::Point(345, 215), cv::Scalar(255), 3, cv::LINE_8, 0);
	cv::circle(img2, cv::Point(320, 240), 25, cv::Scalar(0,0,255), 2, 8, 0);
	cv::line(img2, cv::Point(303, 240), cv::Point(337, 240), cv::Scalar(0,0,255), 1, 8, 0);
	cv::line(img2, cv::Point(320, 257), cv::Point(320, 223), cv::Scalar(0,0,255), 1, 8, 0);
	cv::putText(img2, outString, cv::Point2f(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0,0,255), 1, 4, false);
	//cv::imshow("Result", morph);
	cv::imshow("Result: L", img1);	
	cv::imshow("Result: R", img2);
	if ((cv::waitKey(5) & 255) == 27) {;}}

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
		cv::Mat &Q){

	//cout << "Creating a StereoSGBM object..." << endl;

	//cv::Ptr<cv::StereoBM> stereoBM = cv::StereoBM::create(16, 27);

	cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
		// v needs to be more than 64 (or else wont be wide enough to grab pixels cause cameras are far apart) shifts black borders left and rigth
		-128, // Minimum possible disparity value. Normally it is zero, but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
		256, // Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
		15, // Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
		500, // The first parameter controlling the disparity smoothness. See below.
		1500, // The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*SADWindowSize*SADWindowSize and 32*number_of_image_channels*SADWindowSize*SADWindowSize , respectively).
		-1, // Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
		0, // Truncation value for the prefiltered image pixels. The algorithm first computes x-derivaimg1tive at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
		   // v Making this bigger...finds way less matches...good number is around 5
		0.5, // Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
		500, // Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
		1, // Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.
		cv::StereoSGBM::MODE_HH //Set mode=StereoSGBM::MODE_HH in createStereoSGBM to run the full variant of the algorithm but beware that it may consume a lot of memory.   MODE_SGBM = 0, MODE_HH = 1, MODE_SGBM_3WAY = 2
	); 

	cv::FileStorage fs2(inputXML, cv::FileStorage::READ);
	//cout << "Done" << endl;

	fs2["M1"] >> M1;
	fs2["D1"] >> D1;
	fs2["M2"] >> M2;
	fs2["D2"] >> D2;
	fs2["R"] >> R;
	fs2["T"] >> T;
	fs2["E"] >> E;
	fs2["F"] >> F;

	fs2["R1"] >> R1;
	fs2["R2"] >> R2;
	fs2["P1"] >> P1;
	fs2["P2"] >> P2;

	fs2["map11"] >> map11;
	fs2["map12"] >> map12;
	fs2["map21"] >> map21;
	fs2["map22"] >> map22;

	fs2["Q"] >> Q;
	
	return stereo;
	
}

/*

int main(int argc, char** argv) {

	const char* inputXML = "mar11calib_b.xml";

	cv::Mat M1, M2, D1, D2, R, T, E, F, R1, R2, P1, P2, map11, map12, map21, map22, Q;
	cv::Ptr<cv::StereoSGBM> stereo;
	
	stereo = initializeDepthPerception(
		inputXML, 
		M1, 
		M2, 
		D1, 
		D2,
		R, 
		T,
		E, 
		F, 
		R1, 
		R2,
		P1,
		P2,
		map11,
		map12,
		map21,
		map22,
		Q)
	
	depthPerception(
		stereo,
		frameLeft,
		frameRight		
	);

	return 0;
}
*/