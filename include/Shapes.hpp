#pragma once
#include "Vec3.hpp"
#include <vector>

enum class ShapeType { Sphere, Box, Compound };

struct SphereShape {
    float radius = 0.5f;
};

// Axis-aligned box (no rotation yet — see README "Known Limitations").
struct BoxShape {
    Vec3 halfExtents;
};

// A sub-shape inside a CompoundShape, offset from the owning body's origin.
struct CompoundChild {
    Vec3 localOffset;
    float radius; // compound children are spheres, for simplicity
};

// Represents a body made of several simple primitives instead of one
// convex hull or triangle mesh — e.g. a vehicle hull approximated as a
// handful of spheres along its length. Cheaper to collide against and
// far easier to reason about than mesh/convex colliders.
struct CompoundShape {
    std::vector<CompoundChild> children;
};
