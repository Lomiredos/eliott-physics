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

    // Forme prete pour SAT : soit un polygone convexe (sommets en repere monde),
    // soit un cercle (vertices vide + radius > 0). Les box/OBB sont des polygones
    // a 4 sommets generes depuis l'AABB + la rotation du Transform.
    struct SatShape
    {
        std::vector<ee::math::Vector2<float>> vertices;
        ee::math::Vector2<float> center;
        float radius = 0.0f;
    };

    class PhysicsSystem : public ee::ecs::UpdateSystem
    {
    private:
        QuadTree m_quadTree;
        ee::math::Vector2<float> m_gravity = ee::math::Vector2<float>(0.0f, 981.0f);
        std::unordered_map<ee::ecs::EntityID, ee::math::Rect<float>> m_bounds; // AABB englobante (broad-phase)
        std::unordered_map<ee::ecs::EntityID, SatShape> m_shapes;              // forme SAT de la frame
        std::vector<CollisionPair> m_collisions;

    public:
        PhysicsSystem() : m_quadTree(ee::math::Rect<float>(0.0f, 0.0f, 0.0f, 0.0f), 0) {}

        void configure(ee::math::Rect<float> _worldBounds) { m_quadTree = QuadTree(_worldBounds, 0); }
        void setGravity(ee::math::Vector2<float> _gravity) { m_gravity = _gravity; }

        void update(ee::ecs::World &_world, float _dt) override;

        const std::vector<CollisionPair> &getCollisions() const { return m_collisions; }

    private:
        void resolve(ee::ecs::World &_world, ee::ecs::EntityID _a, ee::ecs::EntityID _b);
    };
}
