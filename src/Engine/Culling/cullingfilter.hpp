#ifndef CULLINGFILTER_H
#define CULLINGFILTER_H

#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <Engine/Renderer/RenderObject/RenderObject.hpp>
#include <Core/Math/LinearAlgebra.hpp>
#include <Core/Math/Math.hpp>

// Solving circular dependency for RenderData
class Renderer;

namespace Ra
{
    namespace Engine
    {

        class Fustrum
        {
        public:
            Fustrum();

            void updateFustrum(const RenderData &data);

            bool intersects(const Core::Aabb aabb);

        private:

            // Points and planes of the Frostrum
            Core::Vector4 m_points[8];
            Core::Vector4 m_planes[6];

        };

        class CullingFilter
        {
        public:
            CullingFilter();

            bool intersectsFustrum(const Core::Aabb aabb);

            void setFustrum(const RenderData &data);

        private:

            Fustrum m_fustrum;
        };

    } // namespace Engine
} // namespace Ra

#endif // CULLINGFILTER_H
