#pragma warning( disable: 4996)

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

using namespace std;

// help function left blank
void help(char * argv[]) {
}

static void StereoCalib(
	const char* imageList,
	int 		nx, // board width
	int			ny, // board height
	const char* calibFile
)

{
	bool 			displayCorners = false;
	bool 			showUndistorted = true;
	bool			isVerticalStereo = false;
	const int		maxScale = 1;
	const float		squareSize = 1.f; //actual square size
	FILE* 			f = fopen(imageList, "rt"); // open text for reading and in text mode
	int				i, j, lr;
	int 			N = nx * ny; // number of dots on board

	vector<string>						imageNames[2];
	vector< cv::Point3f >				boardModel;
	vector< vector<cv::Point3f>> 		objectPoints;
	vector< vector<cv::Point2f>>		points[2];
	vector< cv::Point2f >				corners[2];
	bool								found[2] = { false, false };
	cv::Size							imageSize;

	//cv::Mat R1, R2, P1, P2, map11, map12, map21, map22;

	//READ IN THE TEXT OF CIRCLE GRIDS:

	if (!f) {
		cout << "Cannot open file " << imageList << endl;
		return;
	}

	for (i = 0; i < ny; i++) {
		for (j = 0; j < nx; j++) {
			boardModel.push_back(cv::Point3f((float)(i*squareSize), (float)(j*squareSize), 0.f));
		}
	}

	i = 0;

	for (;;) {
		char buf[1024];
		lr = i % 2;
		if (lr == 0) {
			found[0] = found[1] = false; //found is a 1x2 boolean array reset to 0 every time beginning with left image
			cout << "\n\nResetting found[0] and found[1]" << endl;
		}

		if (!fgets(buf, sizeof(buf) - 3, f)) {
			cout << "Breaking: fgets returned 0" << endl;
			break;
		}

		size_t len = strlen(buf);

		while (len > 0 && isspace(buf[len - 1]))
			buf[--len] = '\0';
		if (buf[0] == '#') {
			cout << "buf[0] == '#'" << endl;
			continue;
		}

		cout << "Image name parsed: " << buf << endl;
		cv::Mat img = cv::imread(buf, 0);
		cout << "Image read into img" << endl;
		//cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
		//imshow("Original Image", img);
		cv::waitKey(0);

		if (img.empty()) {
			cout << "img.empty(), breaking" << endl;
			break;
		}

		imageSize = img.size();
		imageNames[lr].push_back(buf);

		i++;

		//If we did not find board on the left image, won't be on right
		if (lr == 1 && !found[0]) {
			cout << "\nDid not find board on left image, continuing...\n\n" << endl;
			continue;
		}

		//Find circle grids and centers
		for (int s = 1; s <= maxScale; s++) {
			cv::Mat timg = img;
			if (s > 1)
				resize(
					img, // input image.
					timg, // output image; it has the size dsize (when it is non-zero) or the size computed from src.size(), fx, and fy; the type of dst is the same as of src.
					cv::Size(), // output image size; if it equals zero, it is computed as: dsize = Size(round(fx*src.cols), round(fy*src.rows)) Either dsize or both fx and fy must be non - zero.
					s, // scale factor along the horizontal axis; when it equals 0, it is computed as (double)dsize.width / src.cols
					s, // scale factor along the vertical axis; when it equals 0, it is computed as (double)dsize.height / src.rows
					cv::INTER_CUBIC // interpolation method, see cv::InterpolationFlags
				);

			//imshow("timg", timg);

			found[lr] = cv::findCirclesGrid(
				timg,  // grid view of input circles; it must be an 8-bit grayscale or color image.
				cv::Size(nx, ny), // number of circles per row and column ( patternSize = Size(points_per_row, points_per_colum) ).
				corners[lr], // output array of detected centers.
				cv::CALIB_CB_ASYMMETRIC_GRID // uses asymmetric pattern of circles.
				| cv::CALIB_CB_CLUSTERING // uses a special algorithm for grid detection. It is more robust to perspective distortions but much more sensitive to background clutter.
										  //  SimpleBlobDetector::create() blobDetector	feature detector that finds blobs like dark circles on light background.
			);

			if (found[lr] || s == maxScale) {
				if (found[lr])
					cout << "Found found[lr]" << endl;
				else
					cout << "s = maxScale: " << s << endl;
				cv::Mat mcorners(corners[lr]);
				mcorners *= (1. / s);
				//show results of mcorners?
			}
			if (found[lr])
				cout << "found[lr], so breaking" << endl;
			break;
		}

		if (displayCorners) {
			//cout << buf << endl;
			cv::Mat cimg;
			cv::cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
			cout << "converted to color image" << endl;
			//imshow("original cimg", cimg);
			//draw chessboard corners works for circle grids too

			cout << "Drawing chessboard corners" << endl;
			cv::drawChessboardCorners(
				cimg, // Destination image. It must be an 8-bit color image.
				cv::Size(nx, ny), // Number of inner corners per a chessboard row and column (patternSize = cv::Size(points_per_row,points_per_column)).
				corners[lr], // Array of detected corners, the output of findChessboardCorners.
				found[lr] // Parameter indicating whether the complete board was found or not. The return value of findChessboardCorners should be passed here.
			);

			cv::imshow("Display Corners", cimg);

			if ((cv::waitKey(0) & 255) == 27) //allow esc key to quit
				exit(-1);
		}
		else
			cout << '.';

		if (lr == 1 && found[0] && found[1]) {
			cout << "lr == 1 && found[0] && found [1]" << endl;
			objectPoints.push_back(boardModel);
			points[0].push_back(corners[0]);
			points[1].push_back(corners[1]);
		}

	}

	cout << "Closing file..." << endl;
	fclose(f);

	//CALIBRATE THE STEREO CAMERAS
	cv::Mat M1 = cv::Mat::eye(3, 3, CV_64F); // Number of rows, cols, created matrix type
	cv::Mat M2 = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat D1, D2, R, T, E, F;

	cout << "\nRunning stereoCalibrate()...\n";

	cv::stereoCalibrate(
		objectPoints, // Vector of vectors of the calibration pattern points.
		points[0], // Vector of vectors of the projections of the calibration pattern points, observed by the first camera.
		points[1], // Vector of vectors of the projections of the calibration pattern points, observed by the second camera.
		M1, // Input/output first camera matrix: If any of CV_CALIB_USE_INTRINSIC_GUESS , CV_CALIB_FIX_ASPECT_RATIO , CV_CALIB_FIX_INTRINSIC , or CV_CALIB_FIX_FOCAL_LENGTH are specified, some or all of the matrix components must be initialized. See the flags description for details.
		D1, // Input/output vector of distortion coefficients (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]) of 4, 5, 8, 12 or 14 elements. The output vector length depends on the flags.
		M2, // Input/output second camera matrix. The parameter is similar to cameraMatrix1
		D2, // Input/output lens distortion coefficients for the second camera. The parameter is similar to distCoeffs1 .
		imageSize, // Size of the image used only to initialize intrinsic camera matrix.
		R, // Output rotation matrix between the 1st and the 2nd camera coordinate systems.
		T, // Output translation vector between the coordinate systems of the cameras. 
		E, // Output essential matrix.
		F, // Output fundamental matrix.
		   //0,   // Different flags that may be zero or a combination of the following values:
		cv::CALIB_FIX_ASPECT_RATIO |// Optimize f(j)y . Fix the ratio f(j)x/f(j)y
		cv::CALIB_ZERO_TANGENT_DIST |// | // Set tangential distortion coefficients for each camera to zeros and fix there.
		cv::CALIB_SAME_FOCAL_LENGTH |
		cv::CALIB_FIX_FOCAL_LENGTH, // Enforce f(0)x=f(1)x and f(0)y=f(1)y 
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5) // Termination criteria for the iterative optimization algorithm.
	);

	cout << "Done with stereoCalibrate()... \n\n";

	/*
	cv::FileStorage fs2("savedCalib.xml", cv::FileStorage::READ);
	cout << "constructed" << endl;
	cin.get();
	fs2["M1"] >> M1;
	fs2["D1"] >> D1;
	fs2["M2"] >> M2;
	fs2["D2"] >> D2;
	fs2["R"] >> R;
	fs2["T"] >> T;
	fs2["E"] >> E;
	fs2["F"] >> F;

	cout << "Parsed input file" << endl;

	cin.get();
	*/

	/*
	cout << "M1 " << M1 << endl;
	cout << "D1 " << D1 << endl;
	cout << "M2 " << M2 << endl;
	cout << "D2 " << D2 << endl;
	cout << "R " << R << endl;
	cout << "T " << T << endl;
	cout << "E " << E << endl;
	cout << "F " << F << endl;

	cin.get();
	*/


	//CALIBRATION QUALITY CHECK
	/* because the output fundamental matrix implicitly includes all the output information,
	* we can check the quality of calibration using the epipolar geometry constraint:
	* m2^t*F*m1=0
	*/

	cout << "Beginning calibration quality check..." << endl;

	vector<cv::Point3f> lines[2];

	double avgErr = 0;
	int nframes = (int)objectPoints.size();

	for (i = 0; i < nframes; i++) {
		vector<cv::Point2f>& pt0 = points[0][i];
		vector<cv::Point2f>& pt1 = points[1][i];

		/*
		Computes the ideal point coordinates from the observed point coordinates.

		The function is similar to cv::undistort and cv::initUndistortRectifyMap but it operates on a sparse set of points instead of a raster image. Also the function performs a reverse transformation to projectPoints. In case of a 3D object, it does not reconstruct its 3D coordinates, but for a planar object, it does, up to a translation vector, if the proper R is specified.

		For each observed point coordinate (u,v) the function computes:

		x′′←(u−cx)/fxy′′←(v−cy)/fy(x′,y′)=undistort(x′′,y′′,distCoeffs)[XYW]T←R∗[x′y′1]Tx←X/Wy←Y/Wonly performed if P is specified:u′←xf′x+c′xv′←yf′y+c′y
		where undistort is an approximate iterative algorithm that estimates the normalized original point coordinates out of the normalized distorted point coordinates ("normalized" means that the coordinates do not depend on the camera matrix).

		The function can be used for both a stereo camera head or a monocular camera (when R is empty).
		*/
		cout << "Undistorting points of left frame " << i << "..." << endl;

		cv::undistortPoints(
			pt0, // Observed point coordinates, 1xN or Nx1 2-channel (CV_32FC2 or CV_64FC2).
			pt0, //Output ideal point coordinates after undistortion and reverse perspective transformation. If matrix P is identity or omitted, dst will contain normalized point coordinates.
			M1, // Camera matrix ⎡⎣⎢fx000fy0cxcy1⎤⎦⎥ .
			D1, // Input vector of distortion coefficients (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]) of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
			cv::Mat(), //Rectification transformation in the object space (3x3 matrix). R1 or R2 computed by cv::stereoRectify can be passed here. If the matrix is empty, the identity transformation is used.
			M1 // New camera matrix (3x3) or new projection matrix (3x4) ⎡⎣⎢f′x000f′y0c′xc′y1txtytz⎤⎦⎥. P1 or P2 computed by cv::stereoRectify can be passed here. If the matrix is empty, the identity new camera matrix is used.
		);
		cout << "Undistorting points of right frame " << i << "..." << endl;
		cv::undistortPoints(pt1, pt1, M2, D2, cv::Mat(), M2);

		/**
		* For points in an image of a stereo pair, computes the corresponding epilines in the other image.
		**/

		cout << "Computing corresponding epilines of left image..." << endl;

		cv::computeCorrespondEpilines(
			pt0, // Input points. N×1 or 1×N matrix of type CV_32FC2 or vector<Point2f> .
			1, // Index of the image (1 or 2) that contains the points .
			F, // Fundamental matrix that can be estimated using findFundamentalMat or stereoRectify .
			lines[0] //Output vector of the epipolar lines corresponding to the points in the other image. Each line ax+by+c=0 is encoded by 3 numbers (a,b,c) .
		);

		cout << "Computing corresponding epilines of right image..." << endl;

		cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);

		for (j = 0; j < N; j++) {
			double err = fabs(pt0[j].x*lines[1][j].x + pt0[j].y*lines[1][j].y + lines[1][j].z)
				+ fabs(pt1[j].x*lines[0][j].x + pt1[j].y*lines[0][j].y + lines[0][j].z);
			avgErr += err;
		}
	}

	cout << "Avg err = " << avgErr / (nframes*N) << endl;
	cin.get();

	//COMPUTE AND DISPLAY RECTIFICATION
	if (showUndistorted) {
		cv::Mat R1, R2, P1, P2, map11, map12, map21, map22, Q;

		bool useUncalibrated = false;
		//IF BY CALIBRATED (BOUGUET'S METHOD)

		if (!useUncalibrated) {


			/*
			The function computes the rotation matrices for each camera that (virtually)
			make both camera image planes the same plane. Consequently, this makes all the
			epipolar lines parallel and thus simplifies the dense stereo correspondence
			problem. The function takes the matrices computed by stereoCalibrate as input.
			As output, it provides two rotation matrices and also two projection matrices
			in the new coordinates. The function distinguishes the following two cases:

			Horizontal stereo: the first and the second camera views are shifted relative
			to each other mainly along the x axis (with possible small vertical shift). In
			the rectified images, the corresponding epipolar lines in the left and right
			cameras are horizontal and have the same y-coordinate. P1 and P2 look like:
			*/
			cout << "Running stereoRectify()..." << endl;

			stereoRectify(
				M1, // First camera matrix.
				D1, // First camera distortion parameters.
				M2, // Second camera matrix.
				D2, // Second camera distortion parameters.
				imageSize, // Size of the image used for stereo calibration.
				R, // Rotation matrix between the coordinate systems of the first and the second cameras.
				T, // Translation vector between coordinate systems of the cameras.
				R1, // Output 3x3 rectification transform (rotation matrix) for the first camera.
				R2, // Output 3x3 rectification transform (rotation matrix) for the second camera.
				P1, // Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera.
				P2, // Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera.
				Q, // Output 4×4 disparity-to-depth mapping matrix (see reprojectImageTo3D ).
				CV_CALIB_ZERO_DISPARITY//,
									   //0.5//0.9//0 // Operation flags that may be zero or CV_CALIB_ZERO_DISPARITY. If the flag is set, the function makes the principal points of each camera have the same pixel coordinates in the rectified views. And if the flag is not set, the function may still shift the images in the horizontal or vertical direction (depending on the orientation of epipolar lines) to maximize the useful image area.

									   // 	Free scaling parameter. If it is -1 or absent, the function performs the default scaling. Otherwise, the parameter should be between 0 and 1. alpha=0 means that the rectified images are zoomed and shifted so that only valid pixels are visible (no black areas after rectification). alpha=1 means that the rectified image is decimated and shifted so that all the pixels from the original images from the cameras are retained in the rectified images (no source image pixels are lost). Obviously, any intermediate value yields an intermediate result between those two extreme cases.
									   // New image resolution after rectification. The same size should be passed to initUndistortRectifyMap (see the stereo_calib.cpp sample in OpenCV samples directory). When (0,0) is passed (default), it is set to the original imageSize . Setting it to larger value can help you preserve details in the original image, especially when there is a big radial distortion.
									   // Optional output rectangles inside the rectified images where all the pixels are valid. If alpha=0 , the ROIs cover the whole images. Otherwise, they are likely to be smaller (see the picture below).
									   // Optional output rectangles inside the rectified images where all the pixels are valid. If alpha=0 , the ROIs cover the whole images. Otherwise, they are likely to be smaller (see the picture below).

			);

			isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

			cout << "isVerticalStereo: " << isVerticalStereo << endl;
			//precompute maps for cvRemap()


			/*
			Computes the undistortion and rectification transformation map.

			The function computes the joint undistortion and rectification
			transformation and represents the result in the form of maps for
			remap. The undistorted image looks like original, as if it is
			captured with a camera using the camera matrix =newCameraMatrix
			and zero distortion. In case of a monocular camera, newCameraMatrix
			is usually equal to cameraMatrix, or it can be computed by
			cv::getOptimalNewCameraMatrix for a better control over scaling.
			In case of a stereo camera, newCameraMatrix is normally set to P1
			or P2 computed by cv::stereoRectify .

			Also, this new camera is oriented differently in the coordinate
			space, according to R. That, for example, helps to align two heads
			of a stereo camera so that the epipolar lines on both images become
			horizontal and have the same y- coordinate (in case of a horizontally
			aligned stereo camera).

			The function actually builds the maps for the inverse mapping
			algorithm that is used by remap. That is, for each pixel (u,v) in
			the destination (corrected and rectified) image, the function
			computes the corresponding coordinates in the source image (that
			is, in the original image from camera). The following process is
			applied:

			In case of a stereo camera, this function is called twice: once
			for each camera head, after stereoRectify, which in its turn is
			called after cv::stereoCalibrate. But if the stereo camera was
			not calibrated, it is still possible to compute the rectification
			transformations directly from the fundamental matrix using
			cv::stereoRectifyUncalibrated. For each camera, the function
			computes homography H as the rectification transformation in a
			pixel domain, not a rotation matrix R in 3D space. R can be computed
			from H as R=cameraMatrix−1⋅H⋅cameraMatrix where cameraMatrix can be
			chosen arbitrarily.
			*/

			cout << "Initializing undistort rectify map..." << endl;

			initUndistortRectifyMap(
				M1, // Input camera matrix A
				D1, // Input vector of distortion coefficients (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]) of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
				R1, // Optional rectification transformation in the object space (3x3 matrix). R1 or R2 , computed by stereoRectify can be passed here. If the matrix is empty, the identity transformation is assumed. In cvInitUndistortMap R assumed to be an identity matrix.
				P1, // New camera matrix A′
				imageSize, // Undistorted image size.
				CV_16SC2, // Type of the first output map that can be CV_32FC1, CV_32FC2 or CV_16SC2, see cv::convertMaps
				map11, // The first output map.
				map12 // The second output map.
			);
			initUndistortRectifyMap(
				M2,
				D2,
				R2,
				P2,
				imageSize,
				CV_16SC2,
				map21,
				map22
			);

			cout << "Done." << endl;

		}
		else {
			cout << "Hartley's method goes here" << endl;
		}


		//RECTIFY THE IMAGES AND FIND DISPARITY MAPS

		cv::Mat pair;
		if (!isVerticalStereo)
			pair.create(imageSize.height, imageSize.width * 2, CV_8UC3);
		else
			pair.create(imageSize.height * 2, imageSize.width, CV_8UC3);

		// setup for finding stereo correspondences

		cout << "Creating a StereoSGBM object..." << endl;

		cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
			// v needs to be more than 64 (or else wont be wide enough to grab pixels cause cameras are far apart) shifts black borders left and rigth
			-64, // Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
			128, // Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
			11, // Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
			200, // The first parameter controlling the disparity smoothness. See below.
			1000, // The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*SADWindowSize*SADWindowSize and 32*number_of_image_channels*SADWindowSize*SADWindowSize , respectively).
			128, // Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
			0, // Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
			   // v Making this bigger...finds way less matches...good number is around 5
			3, // Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
			200, // Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
			32, // Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.
			cv::StereoSGBM::MODE_SGBM //Set mode=StereoSGBM::MODE_HH in createStereoSGBM to run the full variant of the algorithm but beware that it may consume a lot of memory.   MODE_SGBM = 0, MODE_HH = 1, MODE_SGBM_3WAY = 2
		);

		cout << "Storing to .xml... ";
		cin.get();
		cv::FileStorage fs(calibFile, cv::FileStorage::WRITE);


		fs << "M1" << M1;
		fs << "D1" << D1;
		fs << "M2" << M2;
		fs << "D2" << D2;
		fs << "R" << R;
		fs << "T" << T;
		fs << "E" << E;
		fs << "F" << F;

		fs << "R1" << R1;
		fs << "R2" << R2;
		fs << "P1" << P1;
		fs << "P2" << P2;

		fs << "map11" << map11;
		fs << "map12" << map12;
		fs << "map21" << map21;
		fs << "map22" << map22;

		fs << "Q" << Q;



		cout << "Done" << endl;
		cin.get();

		for (i = 0; i < nframes; i++) {
			cv::Mat img1 = cv::imread(imageNames[0][i].c_str(), 0);
			cv::Mat img2 = cv::imread(imageNames[1][i].c_str(), 0);
			cv::Mat img1r, img2r, disp, vdisp;

			if (img1.empty() || img2.empty())
				continue;

			/*
			Applies a generic geometrical transformation to an image.

			The function remap transforms the source image using the specified map:

			dst(x,y)=src(mapx(x,y),mapy(x,y))

			where values of pixels with non-integer coordinates are computed using one of available interpolation methods. mapx and mapy can be encoded as separate floating-point maps in map1 and map2 respectively, or interleaved floating-point maps of (x,y) in map1, or fixed-point maps created by using convertMaps. The reason you might want to convert from floating to fixed-point representations of a map is that they can yield much faster (2x) remapping operations. In the converted case, map1 contains pairs (cvFloor(x), cvFloor(y)) and map2 contains indices in a table of interpolation coefficients.

			This function cannot operate in-place.
			*/

			cout << "Remapping left image " << i << "..." << endl;

			//cv::imshow("Pre Remap Left Image", img1);
			//cv::imshow("Pre Remap Right Image", img2);
			if ((cv::waitKey() & 255) == 27)
				break;

			cv::remap(
				img1, // Source image. 
				img1r, // Destination image. It has the same size as map1 and the same type as src .
				map11, // The first map of either (x,y) points or just x values having the type CV_16SC2 , CV_32FC1, or CV_32FC2. See convertMaps for details on converting a floating point representation to fixed-point for speed.
				map12, // The second map of y values having the type CV_16UC1, CV_32FC1, or none (empty map if map1 is (x,y) points), respectively.
				cv::INTER_CUBIC //Interpolation method (see cv::InterpolationFlags). The method INTER_AREA is not supported by this function.
								// Pixel extrapolation method (see cv::BorderTypes). When borderMode=BORDER_TRANSPARENT, it means that the pixels in the destination image that corresponds to the "outliers" in the source image are not modified by the function.
								// Value used in case of a constant border. By default, it is 0.
			);
			//cv::imshow("Remapped img1", img1r);
			cout << "Remapping right image " << i << "..." << endl;
			cv::remap(
				img2,
				img2r,
				map21,
				map22,
				cv::INTER_CUBIC
			);
			//cv::imshow("Remapped Left Image", img1r);
			//cv::imshow("Remapped Right Image", img2r);
			if ((cv::waitKey() & 255) == 27)
				break;


			cout << "Done." << endl;

			if (!isVerticalStereo || !useUncalibrated) {


				/* When the stereo camera is oriented vertically, Hartley method does not
				* transpose the image, so the epipolar lines in the rectified
				* images are vertical. Stereo correspondence function does
				* not support such a case.
				*/

				/*
				Computes disparity map for the specified stereo pair.
				*/

				cout << "Running stereo->compute() on remapped pair..." << endl;

				stereo->compute(
					img1r, // Left 8-bit single-channel image.
					img2r, // Right image of the same size and the same type as the left one.
					disp // Output disparity map. It has the same size as the input images. Some algorithms, like StereoBM or StereoSGBM compute 16-bit fixed-point disparity map (where each disparity value has 4 fractional bits), whereas other algorithms output 32-bit floating-point disparity map.
				);

				//cv::imshow("Pre-normalization Disparity Map", disp);
				if ((cv::waitKey() & 255) == 27)
					break;
				/*
				Normalizes the norm or value range of an array.

				The function cv::normalize normalizes scale and shift the input array elements

				The optional mask specifies a sub-array to be normalized. This means that the norm or min-n-max are calculated over the sub-array, and then this sub-array is modified to be normalized. If you want to only use the mask to calculate the norm or min-max but modify the whole array, you can use norm and Mat::convertTo.

				In case of sparse matrices, only the non-zero values are analyzed and transformed. Because of this, the range transformation for sparse matrices is not allowed since it can shift the zero level.
				*/

				cout << "Normalizing..." << endl;

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
				if ((cv::waitKey() & 255) == 27)
					break;
			}

			if (!isVerticalStereo) {
				cv::Mat part = pair.colRange(0, imageSize.width);
				cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
				part = pair.colRange(imageSize.width, imageSize.width * 2);
				cvtColor(img2r, part, cv::COLOR_GRAY2BGR);

				for (j = 0; j < imageSize.height; j += 16)
					cv::line(
						pair,
						cv::Point(0, j),
						cv::Point(imageSize.width * 2, j),
						cv::Scalar(0, 255, 0)
					);

				//cv::imshow("Rectified Pair", pair);
				if ((cv::waitKey() & 255) == 27)
					break;
			}
			else {
				cout << "Vertical Stereo..." << endl;
				cv::Mat part = pair.rowRange(0, imageSize.height);
				cv::cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
				part = pair.rowRange(imageSize.height, imageSize.height * 2);
				cv::cvtColor(img2r, part, cv::COLOR_GRAY2BGR);

				for (j = 0; j < imageSize.width; j += 16) {
					line(
						pair,
						cv::Point(j, 0),
						cv::Point(j, imageSize.height * 2),
						cv::Scalar(0, 255, 0)
					);
				}
				//cv::imshow("Rectified Pair", pair);
				if ((cv::waitKey() & 255) == 27)
					break;
			}
		}
		cout << "Done with everything" << endl;
	}
}

int main(int argc, char** argv) {

	//help(argv);
	int board_w = 4, board_h = 11;
	const char* board_list = "may18.txt";
	const char* calibFile = "may18calibration.xml";

	if (argc == 4) {
		board_list = argv[1];
		board_w = atoi(argv[2]);
		board_h = atoi(argv[3]);
	}

	StereoCalib(
		board_list,
		board_w,
		board_h,
		calibFile
	); //false: Don't use uncalibrated disparity and rectify

	cin.get();

	return 0;
}