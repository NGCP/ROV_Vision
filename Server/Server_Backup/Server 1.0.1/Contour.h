/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Contour.h
 * Author: Khanh Le
 *
 * Created on February 10, 2017, 2:32 AM
 */

#ifndef CONTOUR_H
#define CONTOUR_H
#include "Library.h"

using namespace std;

class Contour {
public:
    Contour(Mat mask);
    Contour(const Contour& orig);
    virtual ~Contour();
    vector<vector<Point> > getContour();
    void FindContourArea();
    bool CompareContourAreas(vector<Point> contour1, vector<Point> contour2);
    void SortContourByArea();
    void PrintContour();
    bool DrawContour(Mat &mask); // if can detect, return true; otherwise, return false
private:
    int numContours;
    vector<vector<Point> > contours;
    double *area;
};

#endif /* CONTOUR_H */

