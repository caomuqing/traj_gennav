//
// Created by lhc on 2019/9/28.
//

#ifndef CUDA_TEST_WS_MATH_STATIC_USE_H
#define CUDA_TEST_WS_MATH_STATIC_USE_H

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <ros/ros.h>
static const float ONE_G = 9.781;

typedef struct fullstate_s{
    ros::Time timestamp;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Matrix3d R;
    fullstate_s() {
        pos.setZero();
        vel.setZero();
        acc.setZero();
        R.setIdentity();
    }
} fullstate_t;

typedef struct flatstate_s{
    ros::Time timestamp;
    Eigen::Vector3d pos;
    double yaw;
    flatstate_s() {
        pos.setZero();
        yaw = 0;
    }
} flatstate_t;

inline float wrapMax(float x, float max) {
    return fmodf(max + fmodf(x, max),max);
}
inline float wrapMinMax(float x, float min, float max) {
    return min + wrapMax(x - min, max - min);
}
inline float wrapPi(float x) {
    return wrapMinMax(x, -M_PI, M_PI);
}


#endif //CUDA_TEST_WS_MATH_STATIC_USE_H
