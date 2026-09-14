#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "ecs/System.hpp"
#include "ecs/World.hpp"
#include "math/Rect.hpp"
#include "math/Transform.hpp"
#include "math/Vector2.hpp"
#include "physics/Collider.hpp"
#include "physics/QuadTree.hpp"
namespace ee::physics {
using CollisionPair = std::pair<ee::ecs::EntityID, ee::ecs::EntityID>;

struct SatShape {
  std::vector<ee::math::Vector2<float>> vertices;
  ee::math::Vector2<float> center;
  float radius = 0.0f;
};

// Construit la forme SAT (sommets en repere monde, box tournee par la
// rotation) d'un collider. Utilisee par PhysicsSystem, et reutilisable
// telle quelle par un rendu debug (meme geometrie que la physique).
SatShape makeShape(const Collider &_col, const ee::math::Transform &_tr);

class PhysicsSystem : public ee::ecs::UpdateSystem {
private:
  QuadTree m_quadTree;
  ee::math::Vector2<float> m_gravity = ee::math::Vector2<float>(0.0f, 981.0f);
  std::unordered_map<ee::ecs::EntityID, ee::math::Rect<float>> m_bounds;
  std::unordered_map<ee::ecs::EntityID, SatShape> m_shapes;
  std::vector<CollisionPair> m_collisions;

public:
  PhysicsSystem()
      : m_quadTree(ee::math::Rect<float>(0.0f, 0.0f, 0.0f, 0.0f), 0) {}

  void configure(ee::math::Rect<float> _worldBounds) {
    m_quadTree = QuadTree(_worldBounds, 0);
  }
  void setGravity(ee::math::Vector2<float> _gravity) { m_gravity = _gravity; }

  void update(ee::ecs::World &_world, float _dt) override;

  const std::vector<CollisionPair> &getCollisions() const {
    return m_collisions;
  }

private:
  void resolve(ee::ecs::World &_world, ee::ecs::EntityID _a,
               ee::ecs::EntityID _b);
};
} // namespace ee::physics
