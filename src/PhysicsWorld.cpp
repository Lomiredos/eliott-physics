#include "physics/PhysicsWorld.hpp"
#include <algorithm>

using namespace ee::physics;
using namespace ee::math;
using namespace ee::ecs;

void ee::physics::PhysicsWorld::update(float _dt)
{
    m_bounds.clear();
    m_quadTree.clear();
    for (ee::ecs::EntityID entity : m_system->m_entities)
    {
        RigidBody &comp = m_world.getComponent<RigidBody>(entity);
        Collider &collidComp = m_world.getComponent<Collider>(entity);
        Transform &transfromComp = m_world.getComponent<Transform>(entity);

        if (comp.isStatic == false)
        {
            comp.velocity += m_gravity * _dt;
            transfromComp.position += comp.velocity * _dt;
        }

        Rect<float> bounds;
        bool isAABB;
        if (std::holds_alternative<AABB>(collidComp.shape))
        {
            AABB &s = std::get<AABB>(collidComp.shape);
            bounds = Rect<float>(transfromComp.position, {s.width, s.height});
            isAABB = true;
        }
        else
        {
            Circle &s = std::get<Circle>(collidComp.shape);
            bounds = Rect<float>(transfromComp.position - Vector2<float>(s.radius, s.radius),
                                 Vector2<float>(s.radius * 2, s.radius * 2));
            isAABB = false;
        }
        m_bounds[entity] = {bounds, isAABB};
        m_quadTree.insert(entity, bounds);
    }

    for (EntityID entity : m_system->m_entities)
    {
        Collider &colliderComp = m_world.getComponent<Collider>(entity);
        Transform &transfromComp = m_world.getComponent<Transform>(entity);

        std::vector<Entry> result = m_quadTree.query(m_bounds[entity].first);
        if (std::holds_alternative<AABB>(colliderComp.shape))
        {
            for (Entry entry : result)
            {
                if (entity == entry.id)
                    continue;
                if (m_bounds[entry.id].second)
                {
                    repulse(entity, entry.id);
                }
                else
                {
                    Collider otherColliderComp = m_world.getComponent<Collider>(entry.id);
                    Transform otherTransfromComp = m_world.getComponent<Transform>(entry.id);
                    Vector2<float> closest;

                    closest.x = std::clamp(otherTransfromComp.position.x, m_bounds[entity].first.getPosition(0).x, m_bounds[entity].first.getPosition(1).x);
                    closest.y = std::clamp(otherTransfromComp.position.y, m_bounds[entity].first.getPosition(0, 0).y, m_bounds[entity].first.getPosition(0, 1).y);

                    if (closest.Distance(entry.bounds.getPosition()) < std::get<Circle>(otherColliderComp.shape).radius)
                    {
                        repulse(entity, entry.id);
                    }
                }
            }
        }
        else
        {
            for (Entry entry : result)
            {
                if (entity == entry.id)
                    continue;
                if (m_bounds[entry.id].second)
                {

                    Collider otherColliderComp = m_world.getComponent<Collider>(entry.id);
                    Transform otherTransfromComp = m_world.getComponent<Transform>(entry.id);
                    Vector2<float> closest;

                    closest.x = std::clamp(m_bounds[entity].first.getPosition().x, otherTransfromComp.position.x, otherTransfromComp.position.x + std::get<AABB>(otherColliderComp.shape).width);
                    closest.y = std::clamp(m_bounds[entity].first.getPosition().y, otherTransfromComp.position.y, otherTransfromComp.position.y + std::get<AABB>(otherColliderComp.shape).height);

                    if (closest.Distance(m_bounds[entity].first.getPosition()) < std::get<Circle>(colliderComp.shape).radius)
                    {
                        repulse(entity, entry.id);
                    }
                }
                else
                {
                    Collider otherColliderComp = m_world.getComponent<Collider>(entry.id);
                    Transform otherTransfromComp = m_world.getComponent<Transform>(entry.id);
                    if (transfromComp.position.Distance(otherTransfromComp.position) <= std::get<Circle>(otherColliderComp.shape).radius + std::get<Circle>(colliderComp.shape).radius)
                    {

                        repulse(entity, entry.id);
                    }
                }
            }
        }
    }
}

void ee::physics::PhysicsWorld::repulse(ee::ecs::EntityID _firstID, ee::ecs::EntityID _secondID)
{
    Transform &firstTransfromComp = m_world.getComponent<Transform>(_firstID);
    Transform &secondTransfromComp = m_world.getComponent<Transform>(_secondID);

    Collider &firstColliderComp = m_world.getComponent<Collider>(_firstID);
    Collider &secondColliderComp = m_world.getComponent<Collider>(_secondID);

    bool firstIsStatic = m_world.getComponent<RigidBody>(_firstID).isStatic;
    bool secondIsStatic = m_world.getComponent<RigidBody>(_secondID).isStatic;

    float firstMove = 0.f;
    float secondMove = 0.f;

    if (firstIsStatic && secondIsStatic)
        return;
    else if (!firstIsStatic && !secondIsStatic)
    {
        firstMove = 0.5f;
        secondMove = 0.5f;
    }
    else if (firstIsStatic == false)
        firstMove = 1.f;
    else if (secondIsStatic == false)
        secondMove = 1.f;

    if (m_bounds[_firstID].second && m_bounds[_secondID].second)
    {
        Rect<float> firstRect = m_bounds[_firstID].first;
        Rect<float> secondRect = m_bounds[_secondID].first;

        float overlapX = std::min(firstRect.getPosition(1).x, secondRect.getPosition(1).x) - std::max(firstRect.getPosition(0).x, secondRect.getPosition(0).x);
        float overlapY = std::min(firstRect.getPosition(1, 1).y, secondRect.getPosition(1, 1).y) - std::max(firstRect.getPosition(0, 0).y, secondRect.getPosition(0, 0).y);

        Vector2<float> deplacement = (overlapX < overlapY ? Vector2<float>(overlapX, 0) : Vector2<float>(0, overlapY));

        deplacement.x *= firstRect.getPosition().x < secondRect.getPosition().x ? 1 : -1;
        deplacement.y *= firstRect.getPosition().y < secondRect.getPosition().y ? 1 : -1;
        
        firstTransfromComp.position -= deplacement * firstMove;
        secondTransfromComp.position += deplacement * secondMove;

    }
    else if (m_bounds[_firstID].second == false && m_bounds[_secondID].second == false)
    {

        Vector2<float> firstPos = firstTransfromComp.position;
        Vector2<float> secondPos = secondTransfromComp.position;

        float firstRadius = std::get<Circle>(firstColliderComp.shape).radius;
        float secondRadius = std::get<Circle>(secondColliderComp.shape).radius;

        Vector2<float> direction = Vector2<float>(secondPos - firstPos).Normalize();

        float distance = firstPos.Distance(secondPos);
        float targetDistance = firstRadius + secondRadius;
        float overlap = targetDistance - distance;

        firstTransfromComp.position -= direction * (overlap * firstMove);
        secondTransfromComp.position += direction * (overlap * secondMove);
    }
    else
    {
        Transform& aabbTrasnfromComp = m_bounds[_firstID].second ? firstTransfromComp : secondTransfromComp;
        Collider& aabbColliderComp = m_bounds[_firstID].second ? firstColliderComp : secondColliderComp;

        Transform& circleTransfromComp = m_bounds[_firstID].second ? secondTransfromComp : firstTransfromComp;
        Collider& circleColliderComp = m_bounds[_firstID].second ? secondColliderComp : firstColliderComp;

        EntityID aabbID = m_bounds[_firstID].second ? _firstID : _secondID;
        EntityID circleID = m_bounds[_firstID].second ? _secondID : _firstID;
        float aabbMove = (aabbID == _firstID) ? firstMove : secondMove;
        float circleMove = (circleID == _firstID) ? firstMove : secondMove;

        Vector2<float> circlePos = circleTransfromComp.position;
        float circleRadius = std::get<Circle>(circleColliderComp.shape).radius;

        AABB rectAABB =std::get<AABB>(aabbColliderComp.shape);

        Vector2<float> aabbDim = Vector2<float>(rectAABB.width, rectAABB.height);
        Rect<float> aabbRect = Rect<float>(aabbTrasnfromComp.position, aabbDim);

        Vector2<float> closest;

        closest.x = std::clamp(circlePos.x, aabbRect.getPosition(0).x, aabbRect.getPosition(1).x);
        closest.y = std::clamp(circlePos.y, aabbRect.getPosition(0, 0).y, aabbRect.getPosition(1, 1).y);
    
        Vector2<float> direction = (circlePos - closest).Normalize();

        float overlap = circleRadius - closest.Distance(circlePos);

        circleTransfromComp.position += direction * overlap * circleMove;
        aabbTrasnfromComp.position -= direction * overlap * aabbMove;
    
    }
}