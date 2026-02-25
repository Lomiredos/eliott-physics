#include "physics/QuadTree.hpp"

void ee::physics::QuadTree::insert(ee::ecs::EntityID _id, const ee::math::Rect<float> &_bounds)
{
    if (NW != nullptr)
    {
        if (NW->m_bounds.contains(_bounds))
            NW->insert(_id, _bounds);
        else if (NE->m_bounds.contains(_bounds))
            NE->insert(_id, _bounds);
        else if (SW->m_bounds.contains(_bounds))
            SW->insert(_id, _bounds);
        else if (SE->m_bounds.contains(_bounds))
            SE->insert(_id, _bounds);
        else
            m_entities.push_back({_id, _bounds});
    }
    else
    {
        m_entities.push_back({_id, _bounds});
        if (m_entities.size() > MAX_OBJECTS && m_actualDeep < MAX_DEPTH)
        {
            subDivise();
            std::vector<Entry> entities = m_entities;
            m_entities.clear();
            for (Entry entry : entities)
            {
                if (NW->m_bounds.contains(entry.bounds))
                    NW->insert(entry.id, entry.bounds);
                else if (NE->m_bounds.contains(entry.bounds))
                    NE->insert(entry.id, entry.bounds);
                else if (SW->m_bounds.contains(entry.bounds))
                    SW->insert(entry.id, entry.bounds);
                else if (SE->m_bounds.contains(entry.bounds))
                    SE->insert(entry.id, entry.bounds);
                else
                    m_entities.push_back(entry);
            }
        }
    }
}

std::vector<ee::physics::Entry> ee::physics::QuadTree::query(const ee::math::Rect<float> &_area)
{
    std::vector<Entry> result;
    for (Entry entity : m_entities)
    {
        if (_area.Intersects(entity.bounds))
            result.push_back(entity);
    }

    if (NW == nullptr)
        return result;

    if (_area.Intersects(NW->m_bounds))
    {
        auto sub = NW->query(_area);
        result.insert(result.end(), sub.begin(), sub.end());
    }
    if (_area.Intersects(NE->m_bounds))
    {
        auto sub = NE->query(_area);
        result.insert(result.end(), sub.begin(), sub.end());
    }

    if (_area.Intersects(SW->m_bounds))

    {
        auto sub = SW->query(_area);
        result.insert(result.end(), sub.begin(), sub.end());
    }
    if (_area.Intersects(SE->m_bounds))
    {
        auto sub = SE->query(_area);
        result.insert(result.end(), sub.begin(), sub.end());
    }

    return result;
}

void ee::physics::QuadTree::clear()
{
    m_entities.clear();
    NE = nullptr;
    NW = nullptr;
    SW = nullptr;
    SE = nullptr;
}

void ee::physics::QuadTree::subDivise()
{
    NW = std::make_unique<QuadTree>(ee::math::Rect<float>(m_bounds.getPosition(0, 0), m_bounds.getSize() / 2), m_actualDeep + 1);

    NE = std::make_unique<QuadTree>(ee::math::Rect<float>(m_bounds.getPosition(0.5, 0), m_bounds.getSize() / 2), m_actualDeep + 1);

    SW = std::make_unique<QuadTree>(ee::math::Rect<float>(m_bounds.getPosition(0, 0.5), m_bounds.getSize() / 2), m_actualDeep + 1);

    SE = std::make_unique<QuadTree>(ee::math::Rect<float>(m_bounds.getPosition(0.5, 0.5), m_bounds.getSize() / 2), m_actualDeep + 1);
}
