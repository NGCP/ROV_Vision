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

#define BUF_LEN 65540 // Larger than maximum UDP packet size

#include "opencv2/opencv.hpp"
using namespace cv;
#include "config.h"

void grabImage(int side, unsigned short servPort) {
    try {
        UDPSocket sock(servPort);

        char buffer[BUF_LEN];
        char longbuf[IMG_SIZE];
        int recvMsgSize;
        string sourceAddress;
        unsigned short sourcePort;

        while (1) {
            for (int i = 0; i < IMG_SIZE/PACK_SIZE; i++) {
                recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
                if (recvMsgSize != PACK_SIZE) {
                    cerr << "Received unexpected size pack:" << recvMsgSize << endl;
                    continue;
                }
                memcpy( & longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
            }
            
            Mat frame = Mat(ROWS, COLS, CV_8UC3, longbuf);
            
            if (side == 0) {
                cout << "Left" << endl;
                imshow("Left", frame);
            }
            else {
                imshow("Right", frame);
            }
            waitKey(20);
        }
    } catch (SocketException & e) {
        cerr << e.what() << endl;
        exit(1);
    }
}

int main(int argc, char * argv[]) {

    if (argc != 3) { // Test for correct number of parameters
        cerr << "Usage: " << argv[0] << " <Server Port> <Server Port>" << endl;
        exit(1);
    }

    unsigned short servPort0 = atoi(argv[1]); // First arg:  local port
    unsigned short servPort1 = atoi(argv[2]);

    namedWindow("Left", CV_WINDOW_AUTOSIZE);
    std::thread thread_1(grabImage, 0, servPort0);
    namedWindow("Right", CV_WINDOW_AUTOSIZE);
    std::thread thread_2(grabImage, 1, servPort1);
    thread_1.join();
    thread_2.join();

    return 0;
}
