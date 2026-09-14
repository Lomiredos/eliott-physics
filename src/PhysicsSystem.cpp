#include "physics/PhysicsSystem.hpp"
#include <algorithm>

using namespace ee::physics;
using namespace ee::math;
using namespace ee::ecs;

namespace
{
    // Annule la composante de vitesse qui rentre dans la collision + isGrounded.
    void killInwardVelocity(RigidBody &_body, Vector2<float> _pushDir)
    {
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
}

void ee::physics::PhysicsSystem::update(ee::ecs::World &_world, float _dt)
{
    m_bounds.clear();
    m_quadTree.clear();
    m_collisions.clear();

    // 1) Integration + calcul des bounds (position = CENTRE, coherent avec le sprite).
    for (EntityID entity : m_entities)
    {
        RigidBody &body = *_world.getComponent<RigidBody>(entity);
        Collider &collider = *_world.getComponent<Collider>(entity);
        Transform &transform = *_world.getComponent<Transform>(entity);

        body.isGrounded = false;

        if (!body.isStatic)
        {
            body.velocity += (m_gravity + body.forces * (1.0f / body.mass)) * _dt;
            transform.position += body.velocity * _dt;
        }

        Vector2<float> center = transform.position + collider.offset;
        Rect<float> bounds;
        bool isAABB;
        if (std::holds_alternative<AABB>(collider.shape))
        {
            AABB &s = std::get<AABB>(collider.shape);
            bounds.setSize(Vector2<float>(s.width, s.height));
            isAABB = true;
        }
        else
        {
            Circle &s = std::get<Circle>(collider.shape);
            bounds.setSize(Vector2<float>(s.radius * 2.0f, s.radius * 2.0f));
            isAABB = false;
        }
        bounds.setPosition(center);
        m_bounds[entity] = {bounds, isAABB};
        m_quadTree.insert(entity, bounds);
    }

    // 2) Detection (chaque paire une seule fois) puis resolution.
    for (EntityID entity : m_entities)
    {
        std::vector<Entry> result = m_quadTree.query(m_bounds[entity].first);
        for (Entry entry : result)
        {
            if (entry.id <= entity)
                continue;

            const Rect<float> &aB = m_bounds[entity].first;
            const Rect<float> &bB = m_bounds[entry.id].first;
            bool aIsAABB = m_bounds[entity].second;
            bool bIsAABB = m_bounds[entry.id].second;

            bool collide = false;
            if (aIsAABB && bIsAABB)
            {
                collide = aB.Intersects(bB);
            }
            else if (!aIsAABB && !bIsAABB)
            {
                float ra = aB.getSize().x * 0.5f;
                float rb = bB.getSize().x * 0.5f;
                collide = aB.getPosition().Distance(bB.getPosition()) <= ra + rb;
            }
            else
            {
                const Rect<float> &box = aIsAABB ? aB : bB;
                const Rect<float> &circ = aIsAABB ? bB : aB;
                float r = circ.getSize().x * 0.5f;
                Vector2<float> c = circ.getPosition();
                Vector2<float> closest;
                closest.x = std::clamp(c.x, box.getPosition(0).x, box.getPosition(1).x);
                closest.y = std::clamp(c.y, box.getPosition(0, 0).y, box.getPosition(1, 1).y);
                collide = closest.Distance(c) < r;
            }

            if (collide)
                repulse(_world, entity, entry.id);
        }
    }
}

void ee::physics::PhysicsSystem::repulse(ee::ecs::World &_world, EntityID _firstID, EntityID _secondID)
{
    m_collisions.push_back({_firstID, _secondID});

    Collider &firstCol = *_world.getComponent<Collider>(_firstID);
    Collider &secondCol = *_world.getComponent<Collider>(_secondID);
    if (firstCol.isSensor || secondCol.isSensor)
        return; // trigger : detecte mais ne repousse pas

    RigidBody &firstBody = *_world.getComponent<RigidBody>(_firstID);
    RigidBody &secondBody = *_world.getComponent<RigidBody>(_secondID);
    Transform &firstTr = *_world.getComponent<Transform>(_firstID);
    Transform &secondTr = *_world.getComponent<Transform>(_secondID);

    if (firstBody.isStatic && secondBody.isStatic)
        return;

    float firstMove = 0.0f;
    float secondMove = 0.0f;
    if (!firstBody.isStatic && !secondBody.isStatic)
    {
        firstMove = 0.5f;
        secondMove = 0.5f;
    }
    else if (!firstBody.isStatic)
        firstMove = 1.0f;
    else
        secondMove = 1.0f;

    const Rect<float> &aB = m_bounds[_firstID].first;
    const Rect<float> &bB = m_bounds[_secondID].first;
    bool aIsAABB = m_bounds[_firstID].second;
    bool bIsAABB = m_bounds[_secondID].second;

    Vector2<float> disp; // deplacement pour separer "first" de "second"

    if (aIsAABB && bIsAABB)
    {
        float overlapX = std::min(aB.getPosition(1).x, bB.getPosition(1).x) - std::max(aB.getPosition(0).x, bB.getPosition(0).x);
        float overlapY = std::min(aB.getPosition(1, 1).y, bB.getPosition(1, 1).y) - std::max(aB.getPosition(0, 0).y, bB.getPosition(0, 0).y);

        if (overlapX < overlapY)
            disp = Vector2<float>(overlapX * (aB.getPosition().x < bB.getPosition().x ? -1.0f : 1.0f), 0.0f);
        else
            disp = Vector2<float>(0.0f, overlapY * (aB.getPosition().y < bB.getPosition().y ? -1.0f : 1.0f));
    }
    else if (!aIsAABB && !bIsAABB)
    {
        float ra = aB.getSize().x * 0.5f;
        float rb = bB.getSize().x * 0.5f;
        Vector2<float> dir = (aB.getPosition() - bB.getPosition()).Normalize();
        float overlap = (ra + rb) - aB.getPosition().Distance(bB.getPosition());
        disp = dir * overlap;
    }
    else
    {
        const Rect<float> &box = aIsAABB ? aB : bB;
        const Rect<float> &circ = aIsAABB ? bB : aB;
        float r = circ.getSize().x * 0.5f;
        Vector2<float> c = circ.getPosition();
        Vector2<float> closest;
        closest.x = std::clamp(c.x, box.getPosition(0).x, box.getPosition(1).x);
        closest.y = std::clamp(c.y, box.getPosition(0, 0).y, box.getPosition(1, 1).y);
        Vector2<float> dirCircle = (c - closest).Normalize();
        float overlap = r - closest.Distance(c);
        disp = (aIsAABB ? dirCircle * -1.0f : dirCircle) * overlap;
    }

    firstTr.position += disp * firstMove;
    secondTr.position -= disp * secondMove;

    if (!firstBody.isStatic)
        killInwardVelocity(firstBody, disp);
    if (!secondBody.isStatic)
        killInwardVelocity(secondBody, disp * -1.0f);
}
