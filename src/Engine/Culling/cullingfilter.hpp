#ifndef CULLINGFILTER_H
#define CULLINGFILTER_H

#include <memory>
#include <vector>

#include <Engine/Renderer/RenderObject/RenderObject.hpp>

#include <Core/Math/LinearAlgebra.hpp>

// Solving circular dependency for RenderData
class Renderer;

namespace Ra
{
    namespace Engine
    {

        class Frostrum
        {
        public:
            Frostrum();

            void updateFrostrum(const RenderData &data);

            bool intersects(const Core::Aabb aabb);

        private:
            // Points of the Frostrum
            Core::Vector3 m_a, m_b, m_c, m_d, m_e, m_f, m_g, m_h;

        };

        class CullingFilter
        {
        public:
            CullingFilter();

            bool intersectsFrostrum(const Core::Aabb aabb);

            void setFrostrum(const RenderData &data);

        private:

            Frostrum m_frostrum;
        };

    } // namespace Engine
} // namespace Ra

#endif // CULLINGFILTER_H
