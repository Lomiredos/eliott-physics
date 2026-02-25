#pragma once
#include <vector>
#include <memory>

#include "math/Rect.hpp"


#include "ecs/EntityManager.hpp"

namespace ee::physics
{
    struct Entry
    {
        ee::ecs::EntityID id;
        ee::math::Rect<float> bounds;
    };

    class QuadTree
    {

    private:
        static constexpr int MAX_OBJECTS = 8;
        static constexpr int MAX_DEPTH = 5;

        ee::math::Rect<float> m_bounds;
        int m_actualDeep;
        std::vector<Entry> m_entities;

        std::unique_ptr<QuadTree> NW;
        std::unique_ptr<QuadTree> NE;
        std::unique_ptr<QuadTree> SW;
        std::unique_ptr<QuadTree> SE;

    public:
        QuadTree(ee::math::Rect<float> _bounds, int _deep) : m_bounds(_bounds), m_actualDeep(_deep) {}
         
        void insert(ee::ecs::EntityID _id, const ee::math::Rect<float>& _bounds);
        std::vector<Entry> query(const ee::math::Rect<float>& _area);
        void clear();

        private:
        void subDivise();
    };

}