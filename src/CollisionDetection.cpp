#include "CollisionDetection.hpp"
#include <algorithm>
#include <cmath>

CollisionInfo Collision::sphereSphere(const RigidBody& a, const RigidBody& b) {
    CollisionInfo info;
    Vec3 delta = b.position - a.position;
    float dist = delta.length();
    float radiusSum = a.sphere.radius + b.sphere.radius;
    if (dist < radiusSum) {
        info.colliding = true;
        info.normal = (dist > 1e-8f) ? delta.normalized() : Vec3(0, 1, 0);
        info.penetration = radiusSum - dist;
    }
    return info;
}

CollisionInfo Collision::boxBox(const RigidBody& a, const RigidBody& b) {
    CollisionInfo info;
    Vec3 delta = b.position - a.position;
    Vec3 overlap(
        a.box.halfExtents.x + b.box.halfExtents.x - std::fabs(delta.x),
        a.box.halfExtents.y + b.box.halfExtents.y - std::fabs(delta.y),
        a.box.halfExtents.z + b.box.halfExtents.z - std::fabs(delta.z)
    );
    if (overlap.x > 0 && overlap.y > 0 && overlap.z > 0) {
        info.colliding = true;
        // Resolve along the axis of least penetration (simple SAT-style pick).
        if (overlap.x < overlap.y && overlap.x < overlap.z) {
            info.normal = Vec3(delta.x < 0 ? -1.0f : 1.0f, 0, 0);
            info.penetration = overlap.x;
        } else if (overlap.y < overlap.z) {
            info.normal = Vec3(0, delta.y < 0 ? -1.0f : 1.0f, 0);
            info.penetration = overlap.y;
        } else {
            info.normal = Vec3(0, 0, delta.z < 0 ? -1.0f : 1.0f);
            info.penetration = overlap.z;
        }
    }
    return info;
}

CollisionInfo Collision::sphereBox(const RigidBody& sphereBody, const RigidBody& boxBody) {
    CollisionInfo info;
    Vec3 delta = sphereBody.position - boxBody.position;
    Vec3 clamped(
        std::max(-boxBody.box.halfExtents.x, std::min(delta.x, boxBody.box.halfExtents.x)),
        std::max(-boxBody.box.halfExtents.y, std::min(delta.y, boxBody.box.halfExtents.y)),
        std::max(-boxBody.box.halfExtents.z, std::min(delta.z, boxBody.box.halfExtents.z))
    );
    Vec3 closestPoint = boxBody.position + clamped;
    Vec3 diff = sphereBody.position - closestPoint;
    float dist = diff.length();
    if (dist < sphereBody.sphere.radius) {
        info.colliding = true;
        // Points from the box toward the sphere.
        info.normal = (dist > 1e-8f) ? diff.normalized() : Vec3(0, 1, 0);
        info.penetration = sphereBody.sphere.radius - dist;
    }
    return info;
}

CollisionInfo Collision::compoundSphere(const RigidBody& compoundBody, const RigidBody& sphereBody) {
    CollisionInfo best;
    for (const auto& child : compoundBody.compound.children) {
        Vec3 childWorldPos = compoundBody.position + child.localOffset;
        Vec3 delta = sphereBody.position - childWorldPos;
        float dist = delta.length();
        float radiusSum = child.radius + sphereBody.sphere.radius;
        if (dist < radiusSum) {
            float penetration = radiusSum - dist;
            if (!best.colliding || penetration > best.penetration) {
                best.colliding = true;
                best.penetration = penetration;
                best.normal = (dist > 1e-8f) ? delta.normalized() : Vec3(0, 1, 0);
            }
        }
    }
    return best;
}

CollisionInfo Collision::compoundBox(const RigidBody& compoundBody, const RigidBody& boxBody) {
    CollisionInfo best;
    for (const auto& child : compoundBody.compound.children) {
        Vec3 childWorldPos = compoundBody.position + child.localOffset;
        Vec3 delta = childWorldPos - boxBody.position;
        Vec3 clamped(
            std::max(-boxBody.box.halfExtents.x, std::min(delta.x, boxBody.box.halfExtents.x)),
            std::max(-boxBody.box.halfExtents.y, std::min(delta.y, boxBody.box.halfExtents.y)),
            std::max(-boxBody.box.halfExtents.z, std::min(delta.z, boxBody.box.halfExtents.z))
        );
        Vec3 closestPoint = boxBody.position + clamped;
        Vec3 diff = childWorldPos - closestPoint;
        float dist = diff.length();
        if (dist < child.radius) {
            float penetration = child.radius - dist;
            if (!best.colliding || penetration > best.penetration) {
                best.colliding = true;
                best.penetration = penetration;
                best.normal = (dist > 1e-8f) ? diff.normalized() : Vec3(0, 1, 0);
            }
        }
    }
    return best;
}

CollisionInfo Collision::detect(const RigidBody& a, const RigidBody& b) {
    if (a.shapeType == ShapeType::Sphere && b.shapeType == ShapeType::Sphere)
        return sphereSphere(a, b);

    if (a.shapeType == ShapeType::Box && b.shapeType == ShapeType::Box)
        return boxBox(a, b);

    if (a.shapeType == ShapeType::Sphere && b.shapeType == ShapeType::Box) {
        CollisionInfo info = sphereBox(a, b);
        info.normal = info.normal * -1.0f; // keep the "A toward B" convention
        return info;
    }
    if (a.shapeType == ShapeType::Box && b.shapeType == ShapeType::Sphere)
        return sphereBox(b, a);

    if (a.shapeType == ShapeType::Compound && b.shapeType == ShapeType::Sphere)
        return compoundSphere(a, b);
    if (a.shapeType == ShapeType::Sphere && b.shapeType == ShapeType::Compound) {
        CollisionInfo info = compoundSphere(b, a);
        info.normal = info.normal * -1.0f;
        return info;
    }

    if (a.shapeType == ShapeType::Compound && b.shapeType == ShapeType::Box) {
        CollisionInfo info = compoundBox(a, b); // returns "box -> compound"
        info.normal = info.normal * -1.0f;      // convert to "A -> B" convention
        return info;
    }
    if (a.shapeType == ShapeType::Box && b.shapeType == ShapeType::Compound)
        return compoundBox(b, a); // already "boxBody(a) -> compoundBody(b)"

    // Compound-vs-compound is not implemented in this mini engine (see README).
    return CollisionInfo{};
}
