#pragma once
#include "Vec3.hpp"
#include "Shapes.hpp"

class RigidBody {
public:
    RigidBody(ShapeType type, float massIn);

    Vec3 position;
    Vec3 velocity;
    Vec3 forceAccum;

    float mass;
    float invMass;      // 0 = static/immovable body
    float restitution = 0.5f;

    ShapeType shapeType;
    SphereShape sphere{};
    BoxShape box{};
    CompoundShape compound{};

    void applyForce(const Vec3& force);
    void applyImpulse(const Vec3& impulse);
    void integrate(float dt);
    bool isStatic() const { return invMass == 0.0f; }
};
