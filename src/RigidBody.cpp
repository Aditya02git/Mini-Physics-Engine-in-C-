#include "RigidBody.hpp"

RigidBody::RigidBody(ShapeType type, float massIn)
    : position(), velocity(), forceAccum(), mass(massIn), shapeType(type) {
    invMass = (massIn <= 0.0f) ? 0.0f : 1.0f / massIn;
}

void RigidBody::applyForce(const Vec3& force) {
    forceAccum += force;
}

void RigidBody::applyImpulse(const Vec3& impulse) {
    velocity += impulse * invMass;
}

void RigidBody::integrate(float dt) {
    if (isStatic()) {
        forceAccum = Vec3();
        return;
    }
    // Semi-implicit (symplectic) Euler: update velocity first, then position
    // using the *new* velocity. Numerically stable at fixed timesteps and
    // what most real-time game physics engines use in practice.
    Vec3 acceleration = forceAccum * invMass;
    velocity += acceleration * dt;
    position += velocity * dt;
    forceAccum = Vec3();
}
