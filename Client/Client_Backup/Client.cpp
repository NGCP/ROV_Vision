/*
 *   C++ UDP socket client for live image upstreaming
 *   Modified from http://cs.ecs.baylor.edu/~donahoo/practical/CSockets/practical/UDPEchoClient.cpp
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

#include "PracticalSocket.h"      // For UDPSocket and SocketException
#include <iostream>               // For cout and cerr
#include <cstdlib>                // For atoi()
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unistd.h>

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <arpa/inet.h>
#include <string.h>

//using namespace std;

#include "opencv2/opencv.hpp"
using namespace cv;
#include "config.h"

extern "C" int xioctl(int fd, int request, void *arg);
extern "C" int print_caps(int fd);
extern "C" int init_mmap(int fd);
extern "C" int capture_image(int fd);
extern "C" int run(int *fd);

uint8_t *buffer0;
uint8_t *buffer1;
std::mutex countLock;

class Barrier {
    private:
        std::mutex mtx;
        std::condition_variable cv;

        size_t m_count;
        const size_t m_init;

        enum State: unsigned char {
            Up, Down
        };
        State m_state;
    public:
        explicit Barrier(std::size_t count) : m_count{count}, m_init(count), m_state{State::Down}{}

        void Sync() {
            std::unique_lock<std::mutex> lock(mtx);

            if (m_state == State::Down) {
                if (--m_count == 0) {
                    m_state = State::Up;
                    cv.notify_all();
                }
                else {
                    cv.wait(lock, [this]{return m_state == State::Up;});
                }
            }
            else {
                if (++m_count == m_init) {
                    m_state = State::Down;
                    cv.notify_all();
                }
                else {
                    cv.wait(lock, [this]{return m_state == State::Down;});
                }
            }
        }
};

Barrier syncher(2);

/*void takePicture(int num, string servAddress, unsigned short servPort) {
    try {
        UDPSocket sock;

        Mat frame;
        VideoCapture cap(num);
        cap.set(CV_CAP_PROP_BUFFERSIZE, 3);
        cap.set(CV_CAP_PROP_FPS, 25);

        if (!cap.isOpened()) {
            std::cerr << "OpenCV Failed to open camera";
            exit(1);
        }

        while (1) {
            syncher.Sync();
            cap.grab();
            cap.retrieve(frame);

            for (int i = 0; i < IMG_SIZE/PACK_SIZE; i++)
                sock.sendTo( & frame.data[i * PACK_SIZE], PACK_SIZE, servAddress, servPort);
            syncher.Sync();
        }
    } catch (SocketException & e) {
        std::cerr << e.what() << endl;
        exit(1);
    }
}*/
 
int xioctl(int fd, int request, void *arg)
{
        int r;
 
        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);
 
        return r;
}
 
int print_caps(int fd)
{
        struct v4l2_capability caps = {};
        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
        {
                perror("Querying Capabilities");
                return 1;
        }
 
        printf( "Driver Caps:\n"
                "  Driver: \"%s\"\n"
                "  Card: \"%s\"\n"
                "  Bus: \"%s\"\n"
                "  Version: %d.%d\n"
                "  Capabilities: %08x\n",
                caps.driver,
                caps.card,
                caps.bus_info,
                (caps.version>>16)&&0xff,
                (caps.version>>24)&&0xff,
                caps.capabilities);
 
 
        struct v4l2_cropcap cropcap = {0};
        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))
        {
                perror("Querying Cropping Capabilities");
                return 1;
        }
 
        printf( "Camera Cropping:\n"
                "  Bounds: %dx%d+%d+%d\n"
                "  Default: %dx%d+%d+%d\n"
                "  Aspect: %d/%d\n",
                cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
                cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
                cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);
 
        int support_grbg10 = 0;
 
        struct v4l2_fmtdesc fmtdesc = {0};
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        char fourcc[5] = {0};
        char c, e;
        printf("  FMT : CE Desc\n--------------------\n");
        while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
        {
                strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
                if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
                    support_grbg10 = 1;
                c = fmtdesc.flags & 1? 'C' : ' ';
                e = fmtdesc.flags & 2? 'E' : ' ';
                printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
                fmtdesc.index++;
        }
        /*
        if (!support_grbg10)
        {
            printf("Doesn't support GRBG10.\n");
            return 1;
        }*/
 
        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = 640;
        fmt.fmt.pix.height = 480;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        
        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
        {
            perror("Setting Pixel Format");
            return 1;
        }
 
        strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
        printf( "Selected Camera Mode:\n"
                "  Width: %d\n"
                "  Height: %d\n"
                "  PixFmt: %s\n"
                "  Field: %d\n",
                fmt.fmt.pix.width,
                fmt.fmt.pix.height,
                fourcc,
                fmt.fmt.pix.field);
        return 0;
}
 
int init_mmap(int fd, int cam)
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
 
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }
 
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 1;
    }
 
    if (cam) {
       buffer1 = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
       printf("Length: %d\nAddress: %p\n", buf.length, buffer1);
       printf("Image Length: %d\n", buf.bytesused);
    } else {
       buffer0 = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
       printf("Length: %d\nAddress: %p\n", buf.length, buffer0);
       printf("Image Length: %d\n", buf.bytesused);
    }
 
    return 0;
}
 
int capture_image(int fd, int cam)
{
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        return 1;
    }
 
    if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return 1;
    }
 
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;
    int r = select(fd+1, &fds, NULL, NULL, &tv);
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return 1;
    }
    if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return 1;
    }
    // printf ("saving image\n");

    /* IplImage* frame;
    CvMat cvmat = cvMat(480, 640, CV_8UC3, (void*)buffer);
    frame = cvDecodeImage(&cvmat, 1);
    //cvNamedWindow("window",CV_WINDOW_AUTOSIZE);
    cvShowImage("window", frame);
    cvWaitKey(1);
    cvSaveImage("image.jpg", frame, 0);
     */ 
    Mat img;
    if (cam) 
    Mat img(480, 640, CV_8UC2, buffer1);
    else 
    Mat img(480, 640, CV_8UC2, buffer0);
    Mat img_p;
    cvtColor(img, img_p, COLOR_YUV2BGR_YUYV);
    imshow("window", img_p);
    waitKey(1);

    return 0;
}
 
int run(int cam, int *fd)
{
        // "/dev/video0"
        if (cam)
        *fd = open("/dev/video1", O_RDWR);
        else
        *fd = open("/dev/video0", O_RDWR);
        if (*fd == -1)
        {
                perror("Opening video device");
                return 1;
        }
        if(print_caps(*fd))
            return 1;
        
        if(init_mmap(*fd, cam))
            return 1;
        return 0;
}

void sendStream(string servAddress, unsigned short servPort, int cam) {
    try {
    UDPSocket sock;
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    int fd = 0;
    uint8_t bufe[5]; 
    run(cam, &fd);
        
    bufe[0] = 215; 
    uint32_t seqNum = 1;
    uint32_t temp;
    clock_t last_cycle = clock();
    while(1) {

    if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        return;
    }
 
    if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return;
    }
 
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 10;
    std::cerr << "Before Sync\n";
    syncher.Sync();
    std::cerr << "After Sync\n";
    int r = select(fd+1, &fds, NULL, NULL, &tv);
    std::cerr << r << std::endl;
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return;
    } else if (r > 0) { 
 
    if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return;
    }

    // printf ("saving image\n");

    /* IplImage* frame;
    CvMat cvmat = cvMat(480, 640, CV_8UC3, (void*)buffer);
    frame = cvDecodeImage(&cvmat, 1);
    //cvNamedWindow("window",CV_WINDOW_AUTOSIZE);
    cvShowImage("window", frame);
    cvWaitKey(1);
    cvSaveImage("image.jpg", frame, 0);
     */ 
    temp = htonl(seqNum);

    std::memcpy(&(bufe[1]), &temp, 4);

    if (cam) {
    //Mat img(480, 640, CV_8UC2, buffer1);
    //Mat img_p;
    //cvtColor(img, img_p, COLOR_YUV2BGR_YUYV);
    //imshow("right", img_p);

        sock.sendTo(bufe, 5, servAddress, servPort);
        for (int i = 0; i < STREAM_SIZE/PACK_SIZE; i++)
            sock.sendTo( &buffer1[i * PACK_SIZE], PACK_SIZE, servAddress, servPort); 
    
   }
    else {
    //Mat img(480, 640, CV_8UC2, buffer0);
    //Mat img_p;
    //cvtColor(img, img_p, COLOR_YUV2BGR_YUYV);
    //imshow("left", img_p);
        sock.sendTo(bufe, 5, servAddress, servPort);
        for (int i = 0; i < STREAM_SIZE/PACK_SIZE; i++)
            sock.sendTo( &buffer0[i * PACK_SIZE], PACK_SIZE, servAddress, servPort); 
    }
    countLock.lock();
    ++seqNum;
    countLock.unlock();
    usleep(1000);
    clock_t next_cycle = clock();
    double duration = (next_cycle - last_cycle) / (double) CLOCKS_PER_SEC;
    std::cout << next_cycle - last_cycle << std::endl;
    std::cout << "\teffective FPS: " << (1/duration) << std::endl;
    last_cycle = next_cycle;
    syncher.Sync();
    //waitKey(1);
	
        //if(capture_image(fd))
        //    return;
        //std::cerr << count++ << std::endl;

    

    }
    }
        // close(fd);
    } catch (SocketException & e) {
        std::cerr << e.what() << endl;
        exit(1);
    }
}


int main(int argc, char * argv[]) {
    if ((argc < 4) || (argc > 4)) { // Test for correct number of arguments
        std::cerr << "Usage: " << argv[0] << " <Server> <Server Port 1> <Server Port 2>\n";
        exit(1);
    }
    uint8_t buff[4];
    int flag = 0;
    string servAddress = argv[1]; // First arg: server address
    unsigned short servPort0 = Socket::resolveService(argv[2], "udp");
    unsigned short servPort1 = Socket::resolveService(argv[3], "udp");
    std::cout << "Port 0: " << servPort0 << "Port 1: " << servPort1 << std::endl;
    
    /*std::thread thread_1(takePicture, 0, servAddress, servPort0);
    std::thread thread_2(takePicture, 1, servAddress, servPort1);
    thread_1.join();
    thread_2.join();
    */ 
    //flag = 1;
    //memcpy(buff, &flag, 4);
    //cout << "send init" << endl;
    //sock.sendTo(buff, 4, servAddress, servPort0);
    //namedWindow("left", CV_WINDOW_AUTOSIZE);
    //namedWindow("right", CV_WINDOW_AUTOSIZE);
    std::thread thread_1(sendStream, servAddress, servPort0, 0);
    std::cerr << "Thread 1 started\n";
    std::thread thread_2(sendStream, servAddress, servPort1, 1);
    std::cerr << "Thread 2 started\n";
    thread_1.join();
    std::cerr << "Thread 1 joined\n";
    thread_2.join();
    std::cerr << "Thread 2 joined\n";
    //sendStream(servAddress, servPort0);
    //munmap(buffer, STREAM_SIZE);
    return 0;
}
