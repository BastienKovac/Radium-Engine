#include "cullingfilter.hpp"

// https://eigen.tuxfamily.org/dox/classEigen_1_1AlignedBox.html#ac252032292dcf60984191e4dd26f0955
// BottomLeft, BottomRight, TopLeft, TopRight. For 3D bounding boxes, the following names are added: BottomLeftCeil, BottomRightCeil, TopLeftCeil, TopRightCeil.


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
            // Using SAT algorithm, which induce some inaccuracy the closer we get to the near plane, as it is used to get intersections between boundary boxes (and frostum isn't a box)
            for(int i = 0; i<3; i++) {
                if(!intersectsDim(i, aabb)) return false;
            }
            
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


