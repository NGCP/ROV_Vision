/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Contour.cpp
 * Author: Khanh Le
 * 
 * Created on February 10, 2017, 2:32 AM
 */

#include "Contour.h"
#include "Library.h"

Contour::Contour(Mat mask) {
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy,
            CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    numContours = contours.size();
    area = new double[numContours];
}

Contour::Contour(const Contour& orig) {
}

Contour::~Contour() {
    delete[] area;
}

void Contour::FindContourArea() {
    for (int i = 0; i < numContours; i++) {
        area[i] = contourArea(contours[i]);
    }
}

void Contour::SortContourByArea() {
    vector<pair < vector<Point>, double>> map(numContours);
    for (int i = 0; i < numContours; i++) {
        map[i] = make_pair(contours[i], area[i]);
    }
    auto sortByArea = []
            (pair<vector<Point>, double>& left,
            pair<vector<Point>, double>& right) {
        return left.second > right.second;
    };

    sort(map.begin(), map.end(), sortByArea);
    for (int i = 0; i < numContours; i++) {
        contours[i] = map[i].first;
        area[i] = map[i].second;
    }

}

/*
void Contour::DrawContour(Mat &img) {
    for (int i = 0; i < numContours; i++) {
        int peri = arcLength(contours[i], true);
        vector<Point> approxCurve;
        //the number of points increases with decreasing epsilon 0.04, 0.05, 0.03
        approxPolyDP(contours[i], approxCurve, 0.04 * peri, true);
        cout << "contour: " << i;
        cout << ", approx Curve: " << approxCurve.size();
        //if (approxCurve.size() >= 5) {
        //drawContours(mask, contours, i, Scalar(0, 255, 0), 2);
        //}

        vector<Point> hull;
        convexHull(contours[i], hull);
        cout << ", area: " << area[i] << ", points: " << contours[i].size();
        double hullArea = contourArea(hull);
        cout << ", hull area: " << hullArea;
        double solidity = area[i] / double(hullArea);
        cout << ", solidity: " << solidity;
        //drawContours(mask, contours, i, Scalar(0, 255, 0), 2);
        Rect rect = boundingRect(contours[i]);
        double rectRatio = double(rect.height) / rect.width;
        cout << ", rect Ratio: " << rectRatio << endl;
        
        if (area[i] > 1000 && 
            approxCurve.size() >= 5 && 
            solidity >= 0.80 &&
            rectRatio >= 0.8 && rectRatio <= 1.2) {
            Point2f center;
            float radius;
            //minEnclosingCircle(contours[i], center, radius);
            minEnclosingCircle(hull, center, radius);

            // find the moments, leave the code out for boosting up performance 
            //Moments mom = moments(contours[i]);
            Moments mom = moments(hull);
            
            Point centroid = Point(static_cast<int> (mom.m10 / mom.m00),
                    static_cast<int> (mom.m01 / mom.m00));

            // draw circle if radius > 10
            if (radius > 5) {
                circle(img, center, int(radius), Scalar(0, 255, 255), 2);
                circle(img, centroid, 5, Scalar(0, 0, 255), -1);
            }
            break;
        }

        // how aout convexity defects
    }
    //drawContours(mask, contours, -1, Scalar(0, 255, 0), 2);           
    imwrite("C:/Users/Khanh Le/Desktop/ROV/Code/mask1.jpg", img);
}
 */

bool Contour::DrawContour(Mat &img) {
    
    int idx = -1;
    int maxVertices = 6;
    vector<Point> maxApproxCurve;
    double aveArea = 0;
    for (int j = 0; j < numContours; j++) {
        aveArea += area[j];
    }
    aveArea /= numContours;
    //cout << "average area: " << aveArea << endl;
    double maxSolidity = 0.85;
    
    // maxVertices = 7, arrpoxCurve = 0.035* peri;
    // maxVertices = 6, arrpoxCurve = 0.0375* peri; rect 0.85 - 1.15
    for (int i = 0; i < numContours; i++) {
        int peri = arcLength(contours[i], true);
        //the number of points increases with decreasing epsilon 0.04, 0.05, 0.03
        vector<Point> approxCurve;
        approxPolyDP(contours[i], approxCurve, 0.035 * peri, true);
        //cout << "contour: " << i;
        //cout << ", approx Curve: " << approxCurve.size();

        vector<Point> hull;
        convexHull(contours[i], hull);
        //cout << ", area: " << area[i] << ", points: " << contours[i].size();
        double hullArea = contourArea(hull);
        //cout << ", hull area: " << hullArea;
        double solidity = area[i] / hullArea;
        //cout << ", solidity: " << solidity;
        //drawContours(mask, contours, i, Scalar(0, 255, 0), 2);
        Rect rect = boundingRect(contours[i]);
        double rectRatio = double(rect.height) / rect.width;
        double extent = area[i]/rect.area(); // must be pi / 4
        //cout << ", rect Ratio: " << rectRatio;
        //cout << ", extent: " << extent << endl;        
        if (area[i] > 600 && // I used 1000
                approxCurve.size() >= maxVertices && approxCurve.size() <= 10 && 
                solidity >= 0.85 && // I used 85
                //extent < 0.79 && 
                rectRatio >= 0.85 && rectRatio <= 1.15) { // I use .85 and 1.15
            
            maxVertices = approxCurve.size();
            maxApproxCurve = approxCurve;
            maxSolidity = solidity;
            idx = i;
        }
    }
    if (idx >= 0) {
        Point2f center;
        float radius;
        minEnclosingCircle(maxApproxCurve, center, radius);
        //minEnclosingCircle(contours[idx], center, radius);

        // find the moments, leave the code out for boosting up performance 
        Moments mom = moments(maxApproxCurve);
        //Moments mom = moments(contours[idx]);

        Point centroid = Point(static_cast<int> (mom.m10 / mom.m00),
                static_cast<int> (mom.m01 / mom.m00));

        // draw circle if radius > 10
        if (radius > 10) {
            circle(img, center, int(radius), Scalar(0, 255, 255), 2);
            circle(img, centroid, 5, Scalar(0, 0, 255), -1);
        }
        return true;
    }
    return false;

    // how aout convexity defects
}

void Contour::PrintContour() {
    cout << "# of contours: " << contours.size() << endl;
    for (int i = 0; i < contours.size(); i++) {
        cout << "contour " << i << ": # of contour points: " << contours[i].size() << endl;

        /*
        for (unsigned int j = 0; j < contours[i].size(); j++) {
            cout << "Point(x,y) = " << contours[i][j] << endl;
        }
         */
        cout << " Area: " << contourArea(contours[i]) << endl;
    }
}
