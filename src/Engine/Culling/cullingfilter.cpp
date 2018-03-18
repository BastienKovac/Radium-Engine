#include "cullingfilter.hpp"

// https://eigen.tuxfamily.org/dox/classEigen_1_1AlignedBox.html#ac252032292dcf60984191e4dd26f0955
// BottomLeft, BottomRight, TopLeft, TopRight. For 3D bounding boxes,
// the following names are added: BottomLeftCeil, BottomRightCeil, TopLeftCeil, TopRightCeil.

// Solving circular dependency for RenderData
#include <Engine/Renderer/Renderer.hpp>
#include <Core/Math/RayCast.hpp>

namespace Ra
{
    namespace Engine
    {

        Frostrum::Frostrum()
        {

        }
        
        Scalar Frostrum::findMax (const int dim, const Core::Aabb aabb) {
            Scalar ans = aabb.corner(Core::Aabb::CornerType::BottomLeft)[dim];
            if(ans < aabb.corner(Core::Aabb::CornerType::BottomRight)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::BottomRight)[dim];
            if(ans < aabb.corner(Core::Aabb::CornerType::TopLeft)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::TopLeft)[dim];
            if(ans < aabb.corner(Core::Aabb::CornerType::TopRight)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::TopRight)[dim];
            if(ans < aabb.corner(Core::Aabb::CornerType::BottomLeftCeil)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::BottomLeftCeil)[dim];
            if(ans < aabb.corner(Core::Aabb::CornerType::BottomRightCeil)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::BottomRightCeil)[dim];
            if(ans < aabb.corner(Core::Aabb::CornerType::TopLeftCeil)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::TopLeftCeil)[dim];
            if(ans < aabb.corner(Core::Aabb::CornerType::TopRightCeil)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::TopRightCeil)[dim];
            
            return ans;
        }
        
        Scalar Frostrum::findMin (const int dim, const Core::Aabb aabb) {
             Scalar ans = aabb.corner(Core::Aabb::CornerType::BottomLeft)[dim];
            if(ans > aabb.corner(Core::Aabb::CornerType::BottomRight)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::BottomRight)[dim];
            if(ans > aabb.corner(Core::Aabb::CornerType::TopLeft)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::TopLeft)[dim];
            if(ans > aabb.corner(Core::Aabb::CornerType::TopRight)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::TopRight)[dim];
            if(ans > aabb.corner(Core::Aabb::CornerType::BottomLeftCeil)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::BottomLeftCeil)[dim];
            if(ans > aabb.corner(Core::Aabb::CornerType::BottomRightCeil)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::BottomRightCeil)[dim];
            if(ans > aabb.corner(Core::Aabb::CornerType::TopLeftCeil)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::TopLeftCeil)[dim];
            if(ans > aabb.corner(Core::Aabb::CornerType::TopRightCeil)[dim])
                ans = aabb.corner(Core::Aabb::CornerType::TopRightCeil)[dim];
            
            return ans;
        }
        
        bool Frostrum::intersectsDim(int i, const Core::Aabb aabb) {
            return (findMin(i, aabb) < max[i]) && (findMin(i, aabb) > min[i]);
            //return true;
        }

        bool Frostrum::intersects(const Core::Aabb aabb)
        {
            // TODO : Intersection algorithm goes here
            // Using SAT algorithm, which induce some inaccuracy the closer we get to the near plane,
            // as it is used to get intersections between boundary boxes (and frostum isn't a box)
            for(int i = 0; i<3; i++) {
                if(!intersectsDim(i, aabb)) return false;
            }

            // If the first test pass, this one use raycast to get rid of the inaccuracy mentioned above
            int count = 0;
            for (int i = 0; i < 6; i++) {
                if(Core::RayCast::vsPlane(new Core::Ray(0.0f, 0.0f, 1.0f), planesOffsets[i], planesNormals[i], NULL))
                    count++;
            }
            //Core::RayCast::vsPlane()
            
            return (count%2 == 1);
        }

        void Frostrum::updateMinMax(Core::Vector4 point) {
            if(point[0] < min[0]) min[0] = point[0];
            if(point[1] < min[1]) min[1] = point[1];
            if(point[2] < min[2]) min[2] = point[2];
            if(point[0] > max[0]) max[0] = point[0];
            if(point[1] > max[1]) max[1] = point[1];
            if(point[2] > max[2]) max[2] = point[2];
        }

        void Frostrum::updateFrostrum(const RenderData &data)
        {
            // TODO : Compute the 8 points of the frostrum from the RenderData
            Core::Matrix4 inverseView = data.viewMatrix.inverse();
            Core::Matrix4 inverseProj = data.projMatrix.inverse();

            m_a = inverseProj * inverseView * Core::Vector4(1.0f,1.0f,1.0f, 1.0f);
            max[0] = m_a[0];
            max[1] = m_a[1];
            max[2] = m_a[2];
            min[0] = m_a[0];
            min[1] = m_a[1];
            min[2] = m_a[2];
            m_b = inverseProj * inverseView *  Core::Vector4f(1.0f,1.0f,-1.0f, 1.0f);
            updateMinMax(m_b);
            m_c = inverseProj * inverseView *  Core::Vector4(1.0f,-1.0f,1.0f, 1.0f);
            updateMinMax(m_c);
            m_d = inverseProj * inverseView *  Core::Vector4(1.0f,-1.0f,-1.0f, 1.0f);
            updateMinMax(m_d);
            m_e = inverseProj * inverseView *  Core::Vector4(-1.0f,1.0f,1.0f, 1.0f);
            updateMinMax(m_e);
            m_f = inverseProj * inverseView *  Core::Vector4(-1.0f,1.0f,-1.0f, 1.0f);
            updateMinMax(m_f);
            m_g = inverseProj * inverseView *  Core::Vector4(-1.0f,-1.0f,1.0f, 1.0f);
            updateMinMax(m_g);
            m_h = inverseProj * inverseView *  Core::Vector4(-1.0f,-1.0f,-1.0f, 1.0f);
            updateMinMax(m_h);
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


