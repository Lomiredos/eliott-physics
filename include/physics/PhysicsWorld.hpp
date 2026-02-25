#pragma once

#include "ecs/World.hpp"
#include "math/Vector2.hpp"
#include "math/Transform.hpp"
#include "physics/QuadTree.hpp"
#include "physics/PhysicsSystem.hpp"
#include "physics/Collider.hpp"
#include "physics/RigidBody.hpp"

namespace ee::physics
{

    class PhysicsWorld
    {

    private:
        std::shared_ptr<PhysicsSystem> m_system;
        ee::ecs::World &m_world;
        QuadTree m_quadTree;
        ee::math::Vector2<float> m_gravity = ee::math::Vector2<float>(0, -1);
        std::unordered_map<ee::ecs::EntityID, std::pair<ee::math::Rect<float>, bool>> m_bounds;

    public:
        PhysicsWorld(ee::ecs::World &_world, ee::math::Rect<float> _worldBounds) : m_world(_world), m_quadTree(_worldBounds, 0)
        {
            m_system = m_world.registerSystem<PhysicsSystem>();

            ee::ecs::Signature sig;
            sig.set(ee::ecs::getComponentID<ee::math::Transform>());
            sig.set(ee::ecs::getComponentID<ee::physics::RigidBody>());
            sig.set(ee::ecs::getComponentID<ee::physics::Collider>());

            m_world.setSystemSignature<PhysicsSystem>(sig);
        }
        void update(float _dt);
        void setGravity(ee::math::Vector2<float> _gravity) { m_gravity = _gravity; }

    private:

    void repulse(ee::ecs::EntityID _firstID, ee::ecs::EntityID _secondID);



    };

}
