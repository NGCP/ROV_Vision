/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Noise.h
 * Author: Khanh Le
 *
 * Created on February 26, 2017, 1:50 AM
 */

#ifndef NOISE_H
#define NOISE_H
#include "Library.h"

class Noise {
public:
    Noise(Mat img);
    Noise(const Noise& orig);
    virtual ~Noise();
    void AddGaussianNoise(Mat &noiseImg, float noisePercent);
    void AddSaltPepperNoise(Mat &noiseImg, float noisePercent);
    void AddUniformNoise(Mat &noiseImg, float noisePercent);
    void AddSwapPixelNoise(Mat &noiseImg, float noisePercent);
    double GetPSNR(const Mat& img1, const Mat& img2);

private:
    Mat img;
    void swap(int &a, int &b);
};

#endif /* NOISE_H */

