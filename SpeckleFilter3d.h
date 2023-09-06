//
// Created by indemind on 2023/8/28.
//

#ifndef RUNOPENCV_SPECKLEFILTER3D_H
#define RUNOPENCV_SPECKLEFILTER3D_H
#include <opencv2/core.hpp>
#include <iostream>
void SpeckleFileter3d(cv::Mat3f& src, cv::Scalar newVal, int areaThred, float thredPtr, int axis);

#endif //RUNOPENCV_SPECKLEFILTER3D_H
