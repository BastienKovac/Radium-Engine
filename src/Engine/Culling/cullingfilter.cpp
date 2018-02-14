#include "cullingfilter.hpp"


namespace Ra
{
    namespace Engine
    {

        Frostrum::Frostrum(const RenderData &data)
        {
            updateCoordinates(data);
        }

        bool Frostrum::intersects(const std::shared_ptr<RenderObject> object)
        {
            // TODO : Intersection algorithm goes here
            return true;
        }

        void Frostrum::updateCoordinates(const RenderData &data)
        {
            // TODO : Compute the 8 points of the frostrum from the RenderData
        }

        CullingFilter::CullingFilter()
        {

        }

        void CullingFilter::addFrostrum(const RenderData &data)
        {
            m_frostrum.push_back(Frostrum(data));
        }

        void CullingFilter::removeFrostrum(int index)
        {
            m_frostrum.erase(m_frostrum.begin() + index);
        }

        void CullingFilter::reset()
        {
            m_frostrum.clear();
        }

        bool CullingFilter::intersectsFrostrums(const std::shared_ptr<RenderObject> object)
        {
            for (auto frostrum : m_frostrum)
            {
                if (!frostrum.intersects(object))
                    return false;
            }
            return true;
        }

    }
}


