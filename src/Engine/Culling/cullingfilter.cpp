#include "cullingfilter.hpp"


namespace Ra
{
    namespace Engine
    {

        Frostrum::Frostrum()
        {

        }

        bool Frostrum::intersects(const Core::Aabb aabb)
        {
            // TODO : Intersection algorithm goes here
            return true;
        }

        void Frostrum::updateFrostrum(const RenderData &data)
        {
            // TODO : Compute the 8 points of the frostrum from the RenderData
        }

        CullingFilter::CullingFilter()
        {

        }

        void CullingFilter::setFrostrum(const RenderData &data)
        {
            m_frostrum.updateFrostrum(data);
        }

        bool CullingFilter::intersectsFrostrum(const Core::Aabb aabb)
        {
            return m_frostrum.intersects(aabb);
        }

    }
}


