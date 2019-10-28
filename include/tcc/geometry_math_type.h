#ifndef GEOMETRY_MATH_TYPE_H_
#define GEOMETRY_MATH_TYPE_H_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <math.h>

template <typename Type, int Size> using Vector = Eigen::Matrix<Type, Size, 1>;

template<class T>
void get_dcm_from_q(Eigen::Matrix<T,3,3> &dcm, const Eigen::Quaternion<T> &q) {
    T a = q.w();
    T b = q.x();
    T c = q.y();
    T d = q.z();
    T aSq = a*a;
    T bSq = b*b;
    T cSq = c*c;
    T dSq = d*d;
    dcm(0, 0) = aSq + bSq - cSq - dSq; 
    dcm(0, 1) = T(2) * (b * c - a * d);
    dcm(0, 2) = T(2) * (a * c + b * d);
    dcm(1, 0) = T(2) * (b * c + a * d);
    dcm(1, 1) = aSq - bSq + cSq - dSq;
    dcm(1, 2) = T(2) * (c * d - a * b);
    dcm(2, 0) = T(2) * (b * d - a * c);
    dcm(2, 1) = T(2) * (a * b + c * d);
    dcm(2, 2) = aSq - bSq - cSq + dSq;
}

template<class T>
void get_q_from_dcm(Eigen::Quaternion<T> &q, const Eigen::Matrix<T,3,3> &dcm) {
    T t = dcm.trace();
    if ( t > T(0) ) {
        t = sqrt(T(1) + t);
        q.w() = T(0.5) * t;
        t = T(0.5) / t;
        q.x() = (dcm(2,1) - dcm(1,2)) * t;
        q.y() = (dcm(0,2) - dcm(2,0)) * t;
        q.z() = (dcm(1,0) - dcm(0,1)) * t;
    } else if (dcm(0,0) > dcm(1,1) && dcm(0,0) > dcm(2,2)) {
        t = sqrt(1.0f + dcm(0,0) - dcm(1,1) - dcm(2,2));
        q.x() = T(0.5) * t;
        t = T(0.5) / t;
        q.w() = (dcm(2,1) - dcm(1,2)) * t;
        q.y() = (dcm(1,0) + dcm(0,1)) * t;
        q.z() = (dcm(0,2) + dcm(2,0)) * t;
    } else if (dcm(1,1) > dcm(2,2)) {
        t = sqrt(T(1.0) - dcm(0,0) + dcm(1,1) - dcm(2,2));
        q.y() = T(0.5) * t;
        t = T(0.5) / t;
        q.w() = (dcm(0,2) - dcm(2,0)) * t;
        q.x() = (dcm(1,0) + dcm(0,1)) * t;
        q.z() = (dcm(2,1) + dcm(1,2)) * t;
    } else {
        t = sqrt(1.0f - dcm(0,0) - dcm(1,1) + dcm(2,2));
        q.z() = T(0.5) * t;
        t = T(0.5) / t;
        q.w() = (dcm(1,0) - dcm(0,1)) * t;
        q.x() = (dcm(0,2) + dcm(2,0)) * t;
        q.y() = (dcm(2,1) + dcm(1,2)) * t;
    }
}

template<class T>
void get_euler_from_R(Vector<T,3> &e, const Eigen::Matrix<T,3,3> &R) {
   T phi = (T)atan2(R(2, 1), R(2, 2));
   T theta = (T)asin(-R(2, 0));
   T psi = (T)atan2(R(1, 0), R(0, 0));
   T pi = T(M_PI);

   if (fabs(theta - pi/T(2.0)) < T(1.0e-3)) {
       phi = T(0.0);
       psi = (T)atan2(R(1, 2), R(0, 2));
   } else if (fabs(theta + pi/T(2.0)) < T(1.0e-3)) {
       phi = T(0.0);
       psi = (T)atan2(-R(1, 2), -R(0, 2));
   }
   e(0) = phi;
   e(1) = theta;
   e(2) = psi;
}

template<class T>
void get_euler_from_q(Vector<T,3> &e, const Eigen::Quaternion<T> &q) {
    Eigen::Matrix<T,3,3> temp_R;
    get_dcm_from_q<T>(temp_R, q);
    get_euler_from_R<T>(e, temp_R);
}

#endif
