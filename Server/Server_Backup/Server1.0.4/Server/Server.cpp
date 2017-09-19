/*
 *   C++ UDP socket server for live image upstreaming
 *   Modified from http://cs.ecs.baylor.edu/~donahoo/practical/CSockets/practical/UDPEchoServer.cpp
 *   Copyright (C) 2015
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "PracticalSocket.h" // For UDPSocket and SocketException
#include <iostream>          // For cout and cerr
#include <cstdlib>           // For atoi()
#include <thread>
#include <unistd.h>
#include <queue>
#include <mutex>
#include <arpa/inet.h>

#include "opencv2/opencv.hpp"
#include "config.h"
#include "ObjectDetection.h"
#include "depthPerception.h"

#define BUF_LEN 65540 // Larger than maximum UDP packet size
#define STREAMING 115 
#define OBJECT_DECT 111
#define DEPTH_PERCP 100 
#define QUEUE_SIZE 30

using namespace cv;

std::mutex mutLock;
std::queue <std::pair <uint32_t, Mat>> q1;
std::queue <std::pair <uint32_t, Mat>> q2;
char ch = 'r';

void grabImage(int side, unsigned short servPort) {
   UDPSocket sock(servPort);
   uint8_t buffer[BUF_LEN];
   uint8_t buf[5];
   char longbuf[IMG_SIZE];
   int recvMsgSize;
   uint32_t seqNum;
   std::pair <uint32_t, Mat> packet1;
   std::pair <uint32_t, Mat> packet2;
   Mat conv0, conv1;
   Mat frame0, frame1;
   char input = STREAMING;
   char temp = STREAMING;
   string sourceAddress;
   unsigned short sourcePort;


   try {

      while (ch != 'e') {
         // Make sure the receive starts at the same point every time, otherwise the images will slide up as frames are received.
         do {
      	   sock.recvFrom(buf, 5, sourceAddress, sourcePort);
         } 
         while (buf[0] != 215);
         
         memmove(&seqNum, (buf + 1), 4);
         
         for (int i = 0; i < IMG_SIZE/PACK_SIZE; i++) {
            recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
            memcpy( & longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
         }

         cerr << "received packets " << endl;

         if (side) { 
            frame0 = Mat(ROWS, COLS, CV_8UC2, longbuf);
            cvtColor(frame0, conv0, COLOR_YUV2BGR_YUYV);
            cerr << "seqNum = " << seqNum << std::endl;
            //packet1.first = ntohl(seqNum);
            packet1.first = seqNum;
            packet1.second = conv0;

            q1.push(packet1);
            cerr << "seqNum = " << seqNum << std::endl;
            // imshow("Left", conv0);
         }
         else {
   	      frame1 = Mat(ROWS, COLS, CV_8UC2, longbuf);
   	      cvtColor(frame1, conv1, COLOR_YUV2BGR_YUYV);
            cerr << "seqNum = " << seqNum << std::endl;

            //packet2.first = ntohl(seqNum);
            packet2.first = seqNum;
            packet2.second = conv1;
            cerr << "seqNum = " << seqNum << std::endl;

            q2.push(packet2);
            // imshow("Right", conv1);
         }
         usleep(5000);
      }
        
   } 

   catch (SocketException & e) {
        cerr << e.what() << endl;
        exit(1);
   }
}

void checkQueue() {
   try {
      std::pair <std::uint32_t, Mat> frameLeft, frameRight;

      if (q1.size() > 0 && q2.size() > 0) {

         frameLeft = q1.front();
         frameRight = q2.front(); 

         while(frameLeft.first != frameRight.first) {
            if(frameLeft.first < frameRight.first) {
               if (q1.size() > 0) {
                  q1.pop();
                  frameLeft = q1.front();
               }
            }
            else if(frameLeft.first > frameRight.first) {
               if (q2.size() > 0) {
                  q2.pop();
                  frameRight = q2.front();
               }
            }
         }
      }
   }
   catch( cv::Exception& e ){
      const char* err_msg = e.what();
      std::cout << "exception caught:" << err_msg << std::endl;
   }
}

void videoStream() {

   std::pair <std::uint32_t, Mat> frameLeft, frameRight;

   // Mat frameLeft, frameRight;
   // namedWindow("Left", CV_WINDOW_AUTOSIZE);
   // namedWindow("Right", CV_WINDOW_AUTOSIZE);

   // Only do something if both queues are not empty
   if (q1.size() > 0 && q2.size() > 0) {

      std::cout << "NOT EMPTY" << endl;
      // Verify that they have the same sequence numbers.
      checkQueue();

      if (q1.size() != q2.size()) {
	      cout << "q1 size/q2 size : "<< q1.size() << "/" << q2.size() << std::endl;
         if(q1.size() > q2.size()){
            q1.pop();
         }
         else {
            q2.pop();
         }
         cout << "q1 size/q2 size : "<< q1.size() << "/" << q2.size() << std::endl;
         
         //usleep(333333);
         return;
      }

      frameLeft = q1.front();
      frameRight = q2.front();

      q1.pop();
      q2.pop();

      imshow("Left", frameLeft.second);
      cv::waitKey(5);
      imshow("Right", frameRight.second);
      cv::waitKey(5);
   }
   else {
      std::cout << "queues were both empty" << std::endl;
   }
   //usleep(33333);
}

void objectDetection() {
   //cout << "object detection" << endl;
   std::pair <std::uint32_t, Mat> frameLeft;
   
   if (q1.size() > 0 && q2.size() > 0) {

      std::cout << "NOT EMPTY" << endl;
      // Verify that they have the same sequence numbers.
      checkQueue();

      if (q1.size() != q2.size()) {
         cout << "q1 size/q2 size : "<< q1.size() << "/" << q2.size() << std::endl;
         if(q1.size() > q2.size()){
            q1.pop();
         }
         else {
            q2.pop();
         }
         cout << "q1 size/q2 size : "<< q1.size() << "/" << q2.size() << std::endl;
         
         //usleep(333333);
         return;
      }

      frameLeft = q1.front();

      q1.pop();
      q2.pop();

      cout << "Running object detection algorithm..." << endl;
      RunObjectDetection(frameLeft.second);
      imshow("Detection Result", frameLeft.second);
   }
   else {
      std::cout << "Queues were both empty" << std::endl;
   }
   //usleep(33333);
}

int main(int argc, char * argv[]) {

   if (argc != 3) { // Test for correct number of parameters
     cerr << "Usage: " << argv[0] << " <Server Port>"  << " <Server Port>" << endl;
     exit(1);
   }

   const char* inputXML = "may18calibration.xml";

   Mat M1, M2, D1, D2, R, T, E, F, R1, R2, P1, P2, map11, map12, map21, map22, Q;
   std::pair <std::uint32_t, Mat> frameLeft, frameRight;

   unsigned short servPort0 = atoi(argv[1]); 
   unsigned short servPort1 = atoi(argv[2]);

   cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
      // v needs to be more than 64 (or else wont be wide enough to grab pixels cause cameras are far apart) shifts black borders left and rigth
      -64, // Minimum possible disparity value. Normally it is zero, but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
      128, // Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
      7, // Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
      200, // The first parameter controlling the disparity smoothness. See below.
      1200, // The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*SADWindowSize*SADWindowSize and 32*number_of_image_channels*SADWindowSize*SADWindowSize , respectively).
      -1, // Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
      0, // Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
         // v Making this bigger...finds way less matches...good number is around 5
      3, // Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
      250, // Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
      1, // Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.
      cv::StereoSGBM::MODE_HH //Set mode=StereoSGBM::MODE_HH in createStereoSGBM to run the full variant of the algorithm but beware that it may consume a lot of memory.   MODE_SGBM = 0, MODE_HH = 1, MODE_SGBM_3WAY = 2
   );

   initializeDepthPerception(
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
   Q);   

   //cout << stereo.minDisparity << endl;

   //cin.get();

   char input = STREAMING;

   std::thread thread_1(grabImage, 0, servPort0);
   std::thread thread_2(grabImage, 1, servPort1);
/*
   for(;;){
      videoStream();
   }
   */

   while(ch != 'e') {
         //moveWindow("Left", 0, 0);
         //moveWindow("Right", 500, 0);
      switch(input) {

         case STREAMING:
            videoStream();
            break;
         case OBJECT_DECT:
            objectDetection();
            break;
         case DEPTH_PERCP:
            cout << "depth perception" << endl;
            if (q1.size() > 0 && q2.size() > 0) {
	            while (q1.size() > 1 && q2.size() > 1) {
	               q1.pop();
	               q2.pop();
               }
               checkQueue();
               frameLeft = q1.front();
 	            frameRight = q2.front();
               q1.pop();
	            q2.pop();
      	      if (q1.size() != q2.size()) 
	               cout << "q1 size/q2 size : "<< q1.size() << "/" << q2.size() << std::endl;
            }
            RunDepthPerception(
               stereo,
               frameLeft.second,
               frameRight.second,
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
               Q);
            break;
      
         default:
            videoStream();
            break;
      }

      ch = waitKey(5);
      switch(ch) {
         case 's':
            input = STREAMING;
            destroyAllWindows();
            break;
         case 'o':
            input = OBJECT_DECT;
            destroyAllWindows();
            break;
         case 'd':
            input = DEPTH_PERCP;
            destroyAllWindows();
            break;
         case 'e':
            break;
         default:
            //destroyAllWindows();
            break;
      }
   }

   // main waits for threads to finish
   thread_1.join();
   thread_2.join();

   return 0;
}
