#ifndef CULLINGFILTER_H
#define CULLINGFILTER_H

#include <memory>
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

        class Frostrum
        {
        public:
            Frostrum();

            void updateFrostrum(const RenderData &data);

            bool intersects(const Core::Aabb aabb);

        private:
            Scalar findMax (const int dim, const Core::Aabb aabb);
            Scalar findMin (const int dim, const Core::Aabb aabb);
            void updateMinMax(Core::Vector4 point);
            bool intersectsDim(int i, const Core::Aabb aabb);
            // Points and planes of the Frostrum
            Core::Vector4 m_a, m_b, m_c, m_d, m_e, m_f, m_g, m_h;
            // Top, bottom, left, right, near, far
            Core::Vector3 planesNormals[6];
            Scalar planesOffsets [6];
            // Min and max values of the frostum for each dimension (shall replace the 8 points above).
            Scalar max[3], min[3];

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
