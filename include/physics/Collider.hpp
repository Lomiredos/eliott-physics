#pragma once

#include <variant>
#include "math/Vector2.hpp"

namespace ee::physics {

    struct AABB{
        float width = 16.f;
        float height = 16.f;
    };

    struct Circle{
        float radius = 16.f;
    };



struct Collider{

    std::variant<Circle, AABB> shape;
    ee::math::Vector2<float> offset = {0.f, 0.f};
    bool isSensor = false;   // true = detecte sans repousser (trigger)


};

}
