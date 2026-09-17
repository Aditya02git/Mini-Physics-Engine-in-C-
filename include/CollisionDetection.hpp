#pragma once
#include "RigidBody.hpp"

struct CollisionInfo {
    bool colliding = false;
    Vec3 normal;        // points from body A toward body B
    float penetration = 0.0f;
};

namespace Collision {
    CollisionInfo sphereSphere(const RigidBody& a, const RigidBody& b);
    CollisionInfo boxBox(const RigidBody& a, const RigidBody& b);
    CollisionInfo sphereBox(const RigidBody& sphereBody, const RigidBody& boxBody);
    CollisionInfo compoundSphere(const RigidBody& compoundBody, const RigidBody& sphereBody);
    CollisionInfo compoundBox(const RigidBody& compoundBody, const RigidBody& boxBody);

    // Dispatches to the correct narrow-phase test based on each body's shape type.
    CollisionInfo detect(const RigidBody& a, const RigidBody& b);
}
