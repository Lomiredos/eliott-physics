#pragma once

#include "ecs/System.hpp"

namespace ee::physics {
    class PhysicsSystem : public ee::ecs::System {
public:
    void update(ee::ecs::World& _world, float _dt){
        return;
    }
    };
}