#pragma once
#include <vector>
#include "RigidBody.hpp"

class PhysicsWorld {
public:
    PhysicsWorld() = default;

    Vec3 gravity{0.0f, -9.81f, 0.0f};
    std::vector<RigidBody*> bodies;

    void addBody(RigidBody* body);
    void step(float dt);

private:
    void applyGravity();
    void integrateBodies(float dt);
    void resolveCollisions();
};
