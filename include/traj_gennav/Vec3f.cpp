/*
Vec3f.inl
Written by Matthew Fisher

Inline file for a 3-dimensional vector of floats
*/

#include "Vec3f.h"
#include <stdlib.h>     /* srand, rand */
#include <math.h>
//#include <iostream>   //cout
#include <cstdio>       //printf

Vec3f::Vec3f()
{

}

// Vec3f::Vec3f(const Vec2f &V, float _z)
// {
//     x = V.x;
//     y = V.y;
//     z = _z;
// }

Vec3f::Vec3f(float _x, float _y, float _z)
{
    x = _x;
    y = _y;
    z = _z;
}

Vec3f::Vec3f(const Vec3f &V)
{
    x = V.x;
    y = V.y;
    z = V.z;
}

// void Vec3f::Vec3f(const RGBColor &c)
// {
//     x = c.r / 255.0f;
//     y = c.g / 255.0f;
//     z = c.b / 255.0f;
// }

Vec3f& Vec3f::operator = (const Vec3f &V)
{
    x = V.x;
    y = V.y;
    z = V.z;
    return *this;
}

Vec3f Vec3f::StdRandomVector()
{
    float x, y, z;
    x = float(rand()) / (RAND_MAX*0.05f) * 2.0f - 1.0f;
    y = float(rand()) / (RAND_MAX*0.05f) * 2.0f - 1.0f;
    z = float(rand()) / (RAND_MAX*0.05f) * 2.0f - 1.0f;
    return Vec3f(x, y, z);
}

Vec3f Vec3f::StdRandomNormal()
{
    return Normalize(StdRandomVector());
}

float Vec3f::Length() const
{
    return sqrtf(x * x + y * y + z * z);
}

float Vec3f::LengthSq() const
{
    return x * x + y * y + z * z;
}

bool Vec3f::Valid() const
{
    return ((x == x) && (y == y) && (z == z));
}

Vec3f Vec3f::Normalize(const Vec3f &V)
{
    float Len = V.Length();
    if(Len == 0.0f)
    {
        return V;
    }
    else
    {
        float Factor = 1.0f / Len;
        return Vec3f(V.x * Factor, V.y * Factor, V.z * Factor);
    }
}

void Vec3f::SetLength(float NewLength)
{
    float Len = Length();
    if(Len != 0.0f)
    {
        float Factor = NewLength / Len;
        x *= Factor;
        y *= Factor;
        z *= Factor;
    }
}

// #ifdef USE_D3D
// Vec3f::operator D3DXVECTOR3() const
// {
//     D3DXVECTOR3 V(x, y, z);
//     return V;
// }
// #endif

/****
PointFromVerticalAngle: generate a point from two points (which form a line) and a vertical angle
the point will have same x-y coordinates as view target
VP: viewpoing, VT: view target, VA: vertical angle
****/
Vec3f Vec3f::PointFromVerticalAngle(const Vec3f &VP, const Vec3f &VT, float VA) 
{   
    Vec3f angles = Vec3f::ComputePitchYaw(VP, VT);
    float h = sqrtf((VT.y-VP.y)*(VT.y-VP.y) + (VT.x - VP.x)*(VT.x - VP.x));// horizontal distance
    Vec3f result(VT.x, VT.y, VP.z+tanf(VA-angles.y)*h);
    //printf("%f %f %f \n", VA_rad, angles.y, tanf(VA_rad-angles.y));
    return result;
}

/****
PointFromVerticalAngle: generate a point from two points (which form a line) and a horizontal angle
the point will have same x-z coordinates as view target
VP: viewpoing, VT: view target, HA: horizontal angle
****/
Vec3f Vec3f::PointFromHorizontalAngle(const Vec3f &VP, const Vec3f &VT, float HA) 
{   
    Vec3f angles = Vec3f::ComputePitchYaw(VP, VT);
    float h = sqrtf((VT.z-VP.z)*(VT.z-VP.z) + (VT.x - VP.x)*(VT.x - VP.x));// horizontal distance
    angles.z = wrappi(angles.z);
    float yawangle = HA+angles.z;
    float pi = 3.1415927;
    float y;
    if (angles.z >=-pi/2 && angles.z < pi/2){
        y = VP.y+tanf(yawangle)*h;
    } else{
        y = VP.y-tanf(yawangle)*h;        
    }
    Vec3f result(VT.x, y, VT.z);
    printf("%f %f %f yawangle = %f\n", result.x, result.y, result.z, yawangle/pi*180);
    return result;
}

float Vec3f::AngleBetween(const Vec3f &Left, const Vec3f &Right)
{
    float LeftLength = Left.Length();
    float RightLength = Right.Length();
    if(LeftLength > 0.0f && RightLength > 0.0f)
    {
        return acosf(clamp(Vec3f::Dot(Left, Right) / LeftLength / RightLength, -1.0f, 1.0f));
    }
    else
    {
        return 0.0f;
    }
}
//atan2( dot(n, cross(x,y)), dot(x,y) )
float Vec3f::AngleBetween2(const Vec3f &Left, const Vec3f &Right)
{   
    Vec3f crossVec = Vec3f::Cross(Left, Right);
    //Vec3f normalVec(0,0,1);
    //float temp = Vec3f::Dot(normalVec,crossVec);
    float temp = (crossVec.z>0?1:-1)*crossVec.Length();
    float tempDot = Vec3f::Dot(Left, Right);
    return atan2f(temp, tempDot);
}

Vec3f Vec3f::ComputePitchYaw(const Vec3f &viewpoint, const Vec3f &target)
{
    Vec3f Result;
    float side = sqrtf((target.y-viewpoint.y)*(target.y-viewpoint.y) + (target.x - viewpoint.x)*(target.x - viewpoint.x));
    Result.x = 0.0;
    Result.y = -atan2f(target.z - viewpoint.z, side);    //pitch angle
    Result.z = atan2f(target.y-viewpoint.y, target.x - viewpoint.x);     //yaw angle
    return Result;
}

Vec3f Vec3f::Cross(const Vec3f &Left, const Vec3f &Right)
{
    Vec3f Result;
    Result.x = Left.y * Right.z - Left.z * Right.y;
    Result.y = Left.z * Right.x - Left.x * Right.z;
    Result.z = Left.x * Right.y - Left.y * Right.x;
    return Result;
}

float Vec3f::Dot(const Vec3f &Left, const Vec3f &Right)
{
    return (Left.x * Right.x + Left.y * Right.y + Left.z * Right.z);
}

Vec3f Vec3f::DirectProduct(const Vec3f &Left, const Vec3f &Right)
{
    return Vec3f(Left.x * Right.x, Left.y * Right.y, Left.z * Right.z);
}

Vec3f Vec3f::Lerp(const Vec3f &Left, const Vec3f &Right, float s)
{
    return (Left + s * (Right - Left));
}

Vec3f Vec3f::Maximize(const Vec3f &Left, const Vec3f &Right)
{
    Vec3f Result = Right;
    if(Left.x > Right.x) Result.x = Left.x;
    if(Left.y > Right.y) Result.y = Left.y;
    if(Left.z > Right.z) Result.z = Left.z;
    return Result;
}

Vec3f Vec3f::Minimize(const Vec3f &Left, const Vec3f &Right)
{
    Vec3f Result = Right;
    if(Left.x < Right.x) Result.x = Left.x;
    if(Left.y < Right.y) Result.y = Left.y;
    if(Left.z < Right.z) Result.z = Left.z;
    return Result;
}

// Vec3f Vec3f::SphericalFromCartesian(const Vec3f &Cartesian)
// {
//     Vec3f Result;
//     Result.x = Cartesian.Length();
//     Result.y = atan2f(Cartesian.y, Cartesian.x);
//     if(Result.x == 0.0f)
//     {
//         Result.z = 0.0f;
//     }
//     else
//     {
//         Result.z = acosf(Cartesian.z / Result.x);
//     }
//     return Result;
// }

// Vec3f Vec3f::CartesianFromSpherical(const Vec3f &Spherical)
// {
//     const float &r = Spherical.x;
//     const float &Theta = Spherical.y;
//     const float &Phi = Spherical.z;
//     float RSinPhi = r * sinf(Phi);
//     return Vec3f(cosf(Theta) * RSinPhi, sinf(Theta) * RSinPhi, r * cosf(Phi));
// }

// bool Vec3f::WithinRect(const Vec3f &Pt, const Rectangle3f &Rect)
// {
//     return((Pt.x >= Rect.Min.x && Pt.x <= Rect.Max.x) &&
//            (Pt.y >= Rect.Min.y && Pt.y <= Rect.Max.y) &&
//            (Pt.z >= Rect.Min.z && Pt.z <= Rect.Max.z));
// }

// Vec3f Vec3f::LinearMap(const Vec3f &s1, const Vec3f &e1, const Vec3f &s2, const Vec3f &e2, const Vec3f &s)
// {
//     return Vec3f(float(Math::LinearMap(s1.x, e1.x, s2.x, e2.x, s.x)),
//                 float(Math::LinearMap(s1.y, e1.y, s2.y, e2.y, s.y)),
//                 float(Math::LinearMap(s1.z, e1.z, s2.z, e2.z, s.z)));
// }

// void Vec3f::CompleteOrthonormalBasis(const Vec3f &Normal, Vec3f &v1, Vec3f &v2)
// {
//     Vec3f AxisTest1 = Cross(Normal, Vec3f::eX);
//     Vec3f AxisTest2 = Cross(Normal, Vec3f::eY);
//     if(AxisTest1.Length() >= AxisTest2.Length())
//     {
//         v1 = Normalize(AxisTest1);
//     }
//     else
//     {
//         v1 = Normalize(AxisTest2);
//     }
//     v2 = Normalize(Cross(v1, Normal));
//     Math::ReorientBasis(v1, v2, Normal);
// }

// Vec3f& Vec3f::operator *= (float Right)
// {
//     x *= Right;
//     y *= Right;
//     z *= Right;
//     return *this;
// }

// Vec3f& Vec3f::operator /= (float Right)
// {
//     x /= Right;
//     y /= Right;
//     z /= Right;
//     return *this;
// }

// Vec3f& Vec3f::operator *= (UINT Right)
// {
//     x *= Right;
//     y *= Right;
//     z *= Right;
//     return *this;
// }

// Vec3f& Vec3f::operator /= (UINT Right)
// {
//     x /= Right;
//     y /= Right;
//     z /= Right;
//     return *this;
// }

Vec3f& Vec3f::operator += (const Vec3f &Right)
{
    x += Right.x;
    y += Right.y;
    z += Right.z;
    return *this;
}

Vec3f& Vec3f::operator -= (const Vec3f &Right)
{
    x -= Right.x;
    y -= Right.y;
    z -= Right.z;
    return *this;
}

Vec3f operator * (const Vec3f &Left, float Right)
{
    return Vec3f(Left.x * Right,
                Left.y * Right,
                Left.z * Right);
}

Vec3f operator * (float Left, const Vec3f &Right)
{
    return Vec3f(Right.x * Left,
                Right.y * Left,
                Right.z * Left);
}

Vec3f operator / (const Vec3f &Left, float Right)
{
    return Vec3f(Left.x / Right,
                Left.y / Right,
                Left.z / Right);
}

Vec3f operator + (const Vec3f &Left, const Vec3f &Right)
{
    return Vec3f(Left.x + Right.x,
                Left.y + Right.y,
                Left.z + Right.z);
}

Vec3f operator - (const Vec3f &Left, const Vec3f &Right)
{
    return Vec3f(Left.x - Right.x,
                Left.y - Right.y,
                Left.z - Right.z);
}

Vec3f operator - (const Vec3f &V)
{
    return Vec3f(-V.x, -V.y, -V.z);
}

// float Vec3f::Dist(const Vec3f &Left, const Vec3f &Right)
// {
//     const float XDiff = Right.x - Left.x;
//     const float YDiff = Right.y - Left.y;
//     const float ZDiff = Right.z - Left.z;
//     return sqrtf(XDiff * XDiff + YDiff * YDiff + ZDiff * ZDiff);
// }

// float Vec3f::DistSq(const Vec3f &Left, const Vec3f &Right)
// {
//     const float XDiff = Right.x - Left.x;
//     const float YDiff = Right.y - Left.y;
//     const float ZDiff = Right.z - Left.z;
//     return (XDiff * XDiff + YDiff * YDiff + ZDiff * ZDiff);
// }

float clamp(float a, float lower, float upper)
{   
    if (lower>upper){
        printf("lower bound is larger than upper bound, something is wrong here\n");
        return 0.0f;
    }
    if (a<lower) a = lower;
    if (a>upper) a = upper;
    return a;
} 
float wrappi(float angle)
{
    float pi = 3.1415927;
    if (angle<-pi){
        return (angle+2*pi);
    } else if (angle>pi){
        return (angle-2*pi);
    } else{
        return angle;
    }
}