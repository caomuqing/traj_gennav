

/*
Plane.cpp
Written by Matthew Fisher

A standard 3D plane (space plane.)  Essentially just the surface defined by a*x + b*y + c*z + d = 0
See Plane.h for a description of these functions.
*/
#include "Plane.h"

Plane::Plane()
{

}

Plane::Plane(const Plane &P)
{
    a = P.a;
    b = P.b;
    c = P.c;
    d = P.d;
}

Plane::Plane(float _a, float _b, float _c, float _d)
{
    a = _a;
    b = _b;
    c = _c;
    d = _d;
}

Plane::Plane(const Vec3f &NormalizedNormal, float _d)
{
    a = NormalizedNormal.x;
    b = NormalizedNormal.y;
    c = NormalizedNormal.z;
    d = _d;
}

Plane Plane::ConstructFromPointNormal(const Vec3f &Pt, const Vec3f &Normal)
{
    Plane Result;
    Vec3f NormalizedNormal = Vec3f::Normalize(Normal);
    Result.a = NormalizedNormal.x;
    Result.b = NormalizedNormal.y;
    Result.c = NormalizedNormal.z;
    Result.d = -Vec3f::Dot(Pt, NormalizedNormal);
    return Result;
}

Plane Plane::ConstructFromPointVectors(const Vec3f &Pt, const Vec3f &V1, const Vec3f &V2)
{
    Vec3f Normal = Vec3f::Cross(V1, V2);
    return ConstructFromPointNormal(Pt, Normal);
}

Plane Plane::Normalize()
{
    Plane Result;
    float Distance = sqrtf(a * a + b * b + c * c);
    Result.a = a / Distance;
    Result.b = b / Distance;
    Result.c = c / Distance;
    Result.d = d / Distance;
    return Result;
}

Plane Plane::ConstructFromPoints(const Vec3f &V0, const Vec3f &V1, const Vec3f &V2)
{
    Vec3f Normal = Vec3f::Normalize(Vec3f::Cross(V1 - V0, V2 - V0));
    return ConstructFromPointNormal(V0, Normal);
}

// Vec3f Plane::IntersectLine(const Line3D &Line) const
// {
//     return IntersectLine(Line.P0, Line.P0 + Line.D);
// }

Vec3f Plane::IntersectLine(const Vec3f &V1, const Vec3f &V2) const
{
    Vec3f Diff = V1 - V2;
    float Denominator = a * Diff.x + b * Diff.y + c * Diff.z;
    if(Denominator == 0.0f)
    {
        return (V1 + V2) * 0.5f;
    }
    float u = (a * V1.x + b * V1.y + c * V1.z + d) / Denominator;

    return (V1 + u * (V2 - V1));
}

Vec3f Plane::IntersectLine(const Vec3f &V1, const Vec3f &V2, bool &Hit) const
{
    Hit = true;
    Vec3f Diff = V2 - V1;
    float denominator = a * Diff.x + b * Diff.y + c * Diff.z;
    if(denominator == 0) {Hit = false; return V1;}
    float u = (a * V1.x + b * V1.y + c * V1.z + d) / denominator;

    return (V1 + u * (V2 - V1));
}

float Plane::IntersectLineRatio(const Vec3f &V1, const Vec3f &V2)
{
    Vec3f Diff = V2 - V1;
    float Denominator = a * Diff.x + b * Diff.y + c * Diff.z;
    if(Denominator == 0.0f)
    {
        return 0.0f;
    }
    return (a * V1.x + b * V1.y + c * V1.z + d) / -Denominator;
}

float Plane::SignedDistance(const Vec3f &Pt) const
{
    return (a * Pt.x + b * Pt.y + c * Pt.z + d);
}

float Plane::UnsignedDistance(const Vec3f &Pt) const
{
    return abs(a * Pt.x + b * Pt.y + c * Pt.z + d);
}

#ifdef USE_D3D
Plane::operator D3DXPLANE()
{
    D3DXPLANE P(a, b, c, d);
    return P;
}
#endif

Vec3f Plane::ClosestPoint(const Vec3f &Point)
{
    return (Point - Normal() * SignedDistance(Point));
}

// bool Plane::PlanePlaneIntersection(const Plane &P1, const Plane &P2, Line3D &L)
// {
//     float Denominator = P1.a * P2.b - P1.b * P2.a;
//     if(Denominator == 0.0f)
//     {
//         // this case should be handled by switching axes...
//         return false;
//     }
//     L.P0 = Vec3f((P2.d * P1.b - P1.d * P2.b) / Denominator, (P1.d * P2.a - P2.d * P1.a) / Denominator, 0.0f);

//     L.D = Vec3f::Cross(P1.Normal(), P2.Normal());
//     if(L.D.Length() == 0.0f)
//     {
//         return false;
//     }
//     L.D = Vec3f::Normalize(L.D);

//     return true;
// }


// float Plane::Dot(const Plane &P, const Vec4f &V)
// {
//     return P.a * V.x + P.b * V.y + P.c * V.z + P.d * V.w;
// }

float Plane::DotCoord(const Plane &P, const Vec3f &V)
{
    return P.a * V.x + P.b * V.y + P.c * V.z + P.d;
}

float Plane::DotNormal(const Plane &P, const Vec3f &V)
{
    return P.a * V.x + P.b * V.y + P.c * V.z;
}
