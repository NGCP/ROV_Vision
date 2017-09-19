/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Utils.h
 * Author: Khanh Le
 *
 * Created on February 9, 2017, 11:03 PM
 */

#ifndef UTILS_H
#define UTILS_H

#include "Library.h"

class Utils {
public:
    Utils();
    Mat Resize(Mat &img, int width, int height);
    Utils(const Utils& orig);
    virtual ~Utils();
private:

};

#endif /* UTILS_H */

