#pragma once
#include <cmath>

// Minimal 3D vector math — deliberately dependency-free.
struct Vec3 {
    float x, y, z;

    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    float lengthSq() const { return dot(*this); }
    float length() const { return std::sqrt(lengthSq()); }

    Vec3 normalized() const {
        float len = length();
        if (len < 1e-8f) return Vec3(0.0f, 0.0f, 0.0f);
        return Vec3(x / len, y / len, z / len);
    }
};
