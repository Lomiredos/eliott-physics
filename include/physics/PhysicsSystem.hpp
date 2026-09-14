#pragma once

#include <vector>
#include <utility>
#include <unordered_map>

#include "ecs/System.hpp"
#include "ecs/World.hpp"
#include "math/Vector2.hpp"
#include "math/Rect.hpp"
#include "math/Transform.hpp"
#include "physics/QuadTree.hpp"
#include "physics/Collider.hpp"
#include "physics/RigidBody.hpp"

namespace ee::physics
{
    using CollisionPair = std::pair<ee::ecs::EntityID, ee::ecs::EntityID>;

    class PhysicsSystem : public ee::ecs::UpdateSystem
    {
    private:
        QuadTree m_quadTree;
        ee::math::Vector2<float> m_gravity = ee::math::Vector2<float>(0.0f, 981.0f);
        std::unordered_map<ee::ecs::EntityID, std::pair<ee::math::Rect<float>, bool>> m_bounds;
        std::vector<CollisionPair> m_collisions;

    public:
        PhysicsSystem() : m_quadTree(ee::math::Rect<float>(0.0f, 0.0f, 0.0f, 0.0f), 0) {}

        void configure(ee::math::Rect<float> _worldBounds) { m_quadTree = QuadTree(_worldBounds, 0); }
        void setGravity(ee::math::Vector2<float> _gravity) { m_gravity = _gravity; }

        void update(ee::ecs::World &_world, float _dt) override;

        const std::vector<CollisionPair> &getCollisions() const { return m_collisions; }

    private:
        void repulse(ee::ecs::World &_world, ee::ecs::EntityID _firstID, ee::ecs::EntityID _secondID);
    };
}
