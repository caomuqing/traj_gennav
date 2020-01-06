

struct Vec3f
{
    Vec3f();
    Vec3f(float _x, float _y, float _z);
    Vec3f(const Vec3f &V);

    float Length() const;
    float LengthSq() const;
    bool Valid() const;
    void SetLength(float NewLength);
    Vec3f DirectProduct(const Vec3f &Left, const Vec3f &Right);
    Vec3f Lerp(const Vec3f &Left, const Vec3f &Right, float s);
    Vec3f Maximize(const Vec3f &Left, const Vec3f &Right);
    Vec3f Minimize(const Vec3f &Left, const Vec3f &Right);

    Vec3f& operator = (const Vec3f &V);
    Vec3f& operator -= (const Vec3f &Right);
    Vec3f& operator += (const Vec3f &Right);

    static Vec3f StdRandomVector();
    static Vec3f StdRandomNormal();
    static Vec3f Normalize(const Vec3f &V);
    static Vec3f Cross(const Vec3f &Left, const Vec3f &Right);
    static float Dot(const Vec3f &Left, const Vec3f &Right);
    static float AngleBetween(const Vec3f &Left, const Vec3f &Right);
    static float AngleBetween2(const Vec3f &Left, const Vec3f &Right);
    static Vec3f ComputePitchYaw(const Vec3f &viewpoint, const Vec3f &target);
    static Vec3f PointFromVerticalAngle(const Vec3f &VP, const Vec3f &VT, float VA);
    static Vec3f PointFromHorizontalAngle(const Vec3f &VP, const Vec3f &VT, float HA);

    float x, y, z;

};

Vec3f operator - (const Vec3f &Left, const Vec3f &Right);
Vec3f operator - (const Vec3f &V);
Vec3f operator + (const Vec3f &Left, const Vec3f &Right);
Vec3f operator * (const Vec3f &Left, float Right);
Vec3f operator * (float Left, const Vec3f &Right);
Vec3f operator / (const Vec3f &Left, float Right);
float clamp(float a, float lower, float upper);
float wrappi(float angle);


