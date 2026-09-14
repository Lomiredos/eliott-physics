#include "physics/PhysicsSystem.hpp"

#include "math/Transform.hpp"
#include "physics/Collider.hpp"
#include "physics/RigidBody.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <optional>

using namespace ee::physics;
using namespace ee::math;
using namespace ee::ecs;

SatShape ee::physics::makeShape(const Collider &_col, const Transform &_tr) {
  SatShape s;
  s.center = _tr.position + _col.offset;

  if (std::holds_alternative<Circle>(_col.shape)) {
    s.radius = std::get<Circle>(_col.shape).radius;
    return s;
  }

  const AABB &box = std::get<AABB>(_col.shape);
  float hw = box.width * 0.5f;
  float hh = box.height * 0.5f;

  float rad = _tr.rotation * 3.14159265f / 180.0f;
  float c = std::cos(rad);
  float sn = std::sin(rad);

  Vector2<float> corners[4] = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
  for (const Vector2<float> &p : corners) {
    Vector2<float> r(p.x * c - p.y * sn, p.x * sn + p.y * c);
    s.vertices.push_back(s.center + r);
  }
  return s;
}

namespace {
bool isCircle(const SatShape &_s) { return _s.vertices.empty(); }

Rect<float> enclosingAABB(const SatShape &_s) {
  Vector2<float> mn;
  Vector2<float> mx;
  if (isCircle(_s)) {
    mn = _s.center - Vector2<float>(_s.radius, _s.radius);
    mx = _s.center + Vector2<float>(_s.radius, _s.radius);
  } else {
    mn = _s.vertices[0];
    mx = _s.vertices[0];
    for (const Vector2<float> &v : _s.vertices) {
      mn.x = std::min(mn.x, v.x);
      mn.y = std::min(mn.y, v.y);
      mx.x = std::max(mx.x, v.x);
      mx.y = std::max(mx.y, v.y);
    }
  }
  return Rect<float>(mn, mx - mn);
}

void projectShape(const SatShape &_s, const Vector2<float> &_axis, float &_min,
                  float &_max) {
  if (isCircle(_s)) {
    float c = _s.center.Dot(_axis);
    _min = c - _s.radius;
    _max = c + _s.radius;
    return;
  }
  _min = _max = _s.vertices[0].Dot(_axis);
  for (size_t i = 1; i < _s.vertices.size(); i++) {
    float d = _s.vertices[i].Dot(_axis);
    _min = std::min(_min, d);
    _max = std::max(_max, d);
  }
}

// axe cercle -> sommet le plus proche du polygone
Vector2<float> circleAxis(const Vector2<float> &_center,
                          const SatShape &_poly) {
  float best = FLT_MAX;
  Vector2<float> bestV = _poly.vertices[0];
  for (const Vector2<float> &v : _poly.vertices) {
    float d = (v - _center).Magnetude();
    if (d < best) {
      best = d;
      bestV = v;
    }
  }
  Vector2<float> a = bestV - _center;
  return a.Magnetude() > 0.0f ? a.Normalize() : Vector2<float>(1.0f, 0.0f);
}

// MTV pour separer A de B (oriente pour pousser A hors de B), nullopt si pas de
// collision.
std::optional<Vector2<float>> satMTV(const SatShape &_a, const SatShape &_b) {
  std::vector<Vector2<float>> axes;

  auto addPolyAxes = [&](const SatShape &s) {
    for (size_t i = 0; i < s.vertices.size(); i++) {
      Vector2<float> e =
          s.vertices[(i + 1) % s.vertices.size()] - s.vertices[i];
      axes.push_back(Vector2<float>(-e.y, e.x).Normalize());
    }
  };

  if (!isCircle(_a))
    addPolyAxes(_a);
  if (!isCircle(_b))
    addPolyAxes(_b);

  if (isCircle(_a) && !isCircle(_b))
    axes.push_back(circleAxis(_a.center, _b));
  if (isCircle(_b) && !isCircle(_a))
    axes.push_back(circleAxis(_b.center, _a));
  if (isCircle(_a) && isCircle(_b)) {
    Vector2<float> d = _b.center - _a.center;
    if (d.Magnetude() <= 0.0f)
      return std::nullopt;
    axes.push_back(d.Normalize());
  }

  if (axes.empty())
    return std::nullopt;

  float minOverlap = FLT_MAX;
  Vector2<float> mtvAxis;
  for (const Vector2<float> &axis : axes) {
    float aMin, aMax, bMin, bMax;
    projectShape(_a, axis, aMin, aMax);
    projectShape(_b, axis, bMin, bMax);
    float overlap = std::min(aMax, bMax) - std::max(aMin, bMin);
    if (overlap <= 0.0f)
      return std::nullopt; // axe separateur -> pas de collision
    if (overlap < minOverlap) {
      minOverlap = overlap;
      mtvAxis = axis;
    }
  }

  // orienter le MTV pour pousser A hors de B
  if ((_a.center - _b.center).Dot(mtvAxis) < 0.0f)
    mtvAxis = mtvAxis * -1.0f;

  return mtvAxis * minOverlap;
}

void killInwardVelocity(RigidBody &_body, Vector2<float> _pushDir) {
  float len = _pushDir.Magnetude();
  if (len <= 0.0f)
    return;
  Vector2<float> normal = _pushDir / len;
  float vn = _body.velocity.Dot(normal);
  if (vn < 0.0f)
    _body.velocity -= normal * vn;
  if (normal.y < -0.5f)
    _body.isGrounded = true;
}
} // namespace

void ee::physics::PhysicsSystem::update(ee::ecs::World &_world, float _dt) {
  m_bounds.clear();
  m_shapes.clear();
  m_collisions.clear();
  m_quadTree.clear();

  // 1) Integration + formes SAT + AABB englobantes (broad-phase).
  for (EntityID entity : m_entities) {
    RigidBody &body = *_world.getComponent<RigidBody>(entity);
    Collider &collider = *_world.getComponent<Collider>(entity);
    Transform &transform = *_world.getComponent<Transform>(entity);

    body.isGrounded = false;

    if (!body.isStatic) {
      body.velocity += (m_gravity + body.forces * (1.0f / body.mass)) * _dt;
      transform.position += body.velocity * _dt;
    }

    SatShape shape = makeShape(collider, transform);
    Rect<float> aabb = enclosingAABB(shape);
    m_bounds[entity] = aabb;
    m_shapes[entity] = shape;
    m_quadTree.insert(entity, aabb);
  }

  // 2) Broad-phase (quadtree) + narrow-phase (SAT) + resolution.
  for (EntityID entity : m_entities) {
    std::vector<Entry> result = m_quadTree.query(m_bounds[entity]);
    for (const Entry &entry : result) {
      if (entry.id <= entity)
        continue;
      resolve(_world, entity, entry.id);
    }
  }
}

void ee::physics::PhysicsSystem::resolve(ee::ecs::World &_world, EntityID _a,
                                         EntityID _b) {
  std::optional<Vector2<float>> mtv = satMTV(m_shapes[_a], m_shapes[_b]);
  if (!mtv)
    return;

  m_collisions.push_back({_a, _b});

  Collider *ca = _world.getComponent<Collider>(_a);
  Collider *cb = _world.getComponent<Collider>(_b);
  if ((ca && ca->isSensor) || (cb && cb->isSensor))
    return; // trigger : detecte, pas de resolution

  RigidBody &ba = *_world.getComponent<RigidBody>(_a);
  RigidBody &bb = *_world.getComponent<RigidBody>(_b);
  if (ba.isStatic && bb.isStatic)
    return;

  float moveA = 0.0f;
  float moveB = 0.0f;
  if (!ba.isStatic && !bb.isStatic) {
    moveA = 0.5f;
    moveB = 0.5f;
  } else if (!ba.isStatic)
    moveA = 1.0f;
  else
    moveB = 1.0f;

  Transform &ta = *_world.getComponent<Transform>(_a);
  Transform &tb = *_world.getComponent<Transform>(_b);

  ta.position += *mtv * moveA;
  tb.position -= *mtv * moveB;

  if (!ba.isStatic)
    killInwardVelocity(ba, *mtv);
  if (!bb.isStatic)
    killInwardVelocity(bb, *mtv * -1.0f);
}
