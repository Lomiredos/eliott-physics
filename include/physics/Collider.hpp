#pragma once

#include <variant>

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


};

}
