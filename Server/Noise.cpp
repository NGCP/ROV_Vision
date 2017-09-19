/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Noise.cpp
 * Author: Khanh Le
 * 
 * Created on February 26, 2017, 1:50 AM
 */

#include "Noise.h"
#include "Library.h"

Noise::Noise(Mat img) {
    this->img = img.clone();
}

Noise::Noise(const Noise& orig) {
}

Noise::~Noise() {
}

void Noise::AddGaussianNoise(Mat &noiseImg, float noisePercent) {
    Mat gaussImg = img.clone();
    int amount = 255 * noisePercent;
    Scalar mean(0, 0, 0);
    Scalar sigma(amount, amount, amount);
    randn(gaussImg, mean, sigma);
    add(img, gaussImg, noiseImg);
}

void Noise::AddSaltPepperNoise(Mat &noiseImg, float noisePercent) {
    int amount = img.rows * img.cols * noisePercent;
    noiseImg = img.clone();
    for (int i = 0; i < amount; i++) {
        int randRow = rand() % img.rows;
        int randCol = rand() % img.cols;
        int randPixel = rand() % 2 == 0 ? 255 : 0;
        noiseImg.at<Vec3b>(randRow, randCol)[0] = randPixel;
        noiseImg.at<Vec3b>(randRow, randCol)[1] = randPixel;
        noiseImg.at<Vec3b>(randRow, randCol)[2] = randPixel;
    }
}

void Noise::AddUniformNoise(Mat &noiseImg, float noisePercent) {
    int amount = img.rows * img.cols * noisePercent;
    noiseImg = img.clone();
    for (int i = 0; i < amount; i++) {
        int randRow = rand() % img.rows;
        int randCol = rand() % img.cols;
        int randPixel = rand() % 256;
        noiseImg.at<Vec3b>(randRow, randCol)[0] = randPixel;
        noiseImg.at<Vec3b>(randRow, randCol)[1] = randPixel;
        noiseImg.at<Vec3b>(randRow, randCol)[2] = randPixel;
    }
}

void Noise::swap(int &a, int &b) {
    a ^= b;
    b ^= a;
    a ^= b;
}

void Noise::AddSwapPixelNoise(Mat &noiseImg, float noisePercent) {
    int amount = img.rows * img.cols * noisePercent;
    noiseImg = img.clone();
    for (int i = 0; i < amount; i++) {
        int randRow1 = rand() % img.rows;
        int randCol1 = rand() % img.cols;
        int randRow2 = rand() % img.rows;
        int randCol2 = rand() % img.cols;

        Vec3b intensity1 = img.at<Vec3b>(randRow1, randCol1);
        Vec3b intensity2 = img.at<Vec3b>(randRow2, randCol2);

        noiseImg.at<Vec3b>(randRow1, randCol1) = intensity2;
        noiseImg.at<Vec3b>(randRow2, randCol2) = intensity1;
      }
}

double Noise::GetPSNR(const Mat& img1, const Mat& img2) {
    Mat s1;
    absdiff(img1, img2, s1); // |I1 - I2|
    s1.convertTo(s1, CV_32F); // cannot make a square on 8 bits
    s1 = s1.mul(s1); // |I1 - I2|^2

    Scalar s = sum(s1); // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if (sse <= 1e-10) { // for small values return zero
        return 0;
    }

    double mse = sse / double(img1.channels() * img1.total());
    double psnr = 10.0 * log10((255 * 255) / mse);
    return psnr;

}
