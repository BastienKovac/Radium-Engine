#include "cullingfilter.hpp"

// https://eigen.tuxfamily.org/dox/classEigen_1_1AlignedBox.html#ac252032292dcf60984191e4dd26f0955
// BottomLeft, BottomRight, TopLeft, TopRight. For 3D bounding boxes,
// the following names are added: BottomLeftCeil, BottomRightCeil, TopLeftCeil, TopRightCeil.

// Solving circular dependency for RenderData
#include <Engine/Renderer/Renderer.hpp>

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
            
            return true;
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
            if(m_b[0] < min[0]) min[0] = m_b[0];
            if(m_b[1] < min[1]) min[1] = m_b[1];
            if(m_b[2] < min[2]) min[2] = m_b[2];
            if(m_b[0] < min[0]) min[0] = m_b[0];
            if(m_b[1] < min[1]) min[1] = m_b[1];
            if(m_b[2] < min[2]) min[2] = m_b[2];
            m_c = inverseProj * inverseView *  Core::Vector4(1.0f,-1.0f,1.0f, 1.0f);
            if(m_c[0] < min[0]) min[0] = m_c[0];
            if(m_c[1] < min[1]) min[1] = m_c[1];
            if(m_c[2] < min[2]) min[2] = m_c[2];
            if(m_c[0] < min[0]) min[0] = m_c[0];
            if(m_c[1] < min[1]) min[1] = m_c[1];
            if(m_c[2] < min[2]) min[2] = m_c[2];
            m_d = inverseProj * inverseView *  Core::Vector4(1.0f,-1.0f,-1.0f, 1.0f);
            if(m_d[0] < min[0]) min[0] = m_d[0];
            if(m_d[1] < min[1]) min[1] = m_d[1];
            if(m_d[2] < min[2]) min[2] = m_d[2];
            if(m_d[0] < min[0]) min[0] = m_d[0];
            if(m_d[1] < min[1]) min[1] = m_d[1];
            if(m_d[2] < min[2]) min[2] = m_d[2];
            m_e = inverseProj * inverseView *  Core::Vector4(-1.0f,1.0f,1.0f, 1.0f);
            if(m_e[0] < min[0]) min[0] = m_e[0];
            if(m_e[1] < min[1]) min[1] = m_e[1];
            if(m_e[2] < min[2]) min[2] = m_e[2];
            if(m_e[0] < min[0]) min[0] = m_e[0];
            if(m_e[1] < min[1]) min[1] = m_e[1];
            if(m_e[2] < min[2]) min[2] = m_e[2];
            m_f = inverseProj * inverseView *  Core::Vector4(-1.0f,1.0f,-1.0f, 1.0f);
            if(m_f[0] < min[0]) min[0] = m_f[0];
            if(m_f[1] < min[1]) min[1] = m_f[1];
            if(m_f[2] < min[2]) min[2] = m_f[2];
            if(m_f[0] < min[0]) min[0] = m_f[0];
            if(m_f[1] < min[1]) min[1] = m_f[1];
            if(m_f[2] < min[2]) min[2] = m_f[2];
            m_g = inverseProj * inverseView *  Core::Vector4(-1.0f,-1.0f,1.0f, 1.0f);
            if(m_g[0] < min[0]) min[0] = m_g[0];
            if(m_g[1] < min[1]) min[1] = m_g[1];
            if(m_g[2] < min[2]) min[2] = m_g[2];
            if(m_g[0] < min[0]) min[0] = m_g[0];
            if(m_g[1] < min[1]) min[1] = m_g[1];
            if(m_g[2] < min[2]) min[2] = m_g[2];
            m_h = inverseProj * inverseView *  Core::Vector4(-1.0f,-1.0f,-1.0f, 1.0f);
            if(m_h[0] < min[0]) min[0] = m_h[0];
            if(m_h[1] < min[1]) min[1] = m_h[1];
            if(m_h[2] < min[2]) min[2] = m_h[2];
            if(m_h[0] < min[0]) min[0] = m_h[0];
            if(m_h[1] < min[1]) min[1] = m_h[1];
            if(m_h[2] < min[2]) min[2] = m_h[2];
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


