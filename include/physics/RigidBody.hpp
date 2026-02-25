#pragma once
#include "math/Vector2.hpp"

namespace ee::physics
{
    struct RigidBody
    {
        ee::math::Vector2<float> velocity;
        ee::math::Vector2<float> forces;
        float mass = 1.0f;
        bool isStatic = false;
    };

}
