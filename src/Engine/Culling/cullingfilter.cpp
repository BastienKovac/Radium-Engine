#include "cullingfilter.hpp"

// https://eigen.tuxfamily.org/dox/classEigen_1_1AlignedBox.html#ac252032292dcf60984191e4dd26f0955
// BottomLeft, BottomRight, TopLeft, TopRight. For 3D bounding boxes,
// the following names are added: BottomLeftCeil, BottomRightCeil, TopLeftCeil, TopRightCeil.

// Solving circular dependency for RenderData
#include <Engine/Renderer/Renderer.hpp>
#include <Core/Math/RayCast.hpp>

#include <Engine/System/System.hpp>
#include <Engine/Managers/SystemDisplay/SystemDisplay.hpp>

namespace Ra
{
    namespace Engine
    {

        Frostrum::Frostrum()
        {

        }
        
        Scalar Frostrum::findMax (const int dim, const Core::Aabb aabb)
        {
            return aabb.max()[dim];
        }
        
        Scalar Frostrum::findMin (const int dim, const Core::Aabb aabb)
        {
            return aabb.min()[dim];
        }
        
        bool Frostrum::intersectsDim(int i, const Core::Aabb aabb)
        {
            // Step of the SAT algorithm, handle a single dimension
            return (findMin(i, aabb) < max[i]) && (findMax(i, aabb) > min[i]);
        }

        bool Frostrum::intersects(const Core::Aabb aabb)
        {
            // TODO : Intersection algorithm goes here
            // Using SAT algorithm, which induce some inaccuracy the closer we get to the near plane,
            // as it is used to get intersections between boundary boxes (and frostum isn't a box)
            return intersectsDim(0, aabb) && intersectsDim(1, aabb) && intersectsDim(2, aabb);

            /*// If the first test pass, this one use raycast to get rid of the inaccuracy mentioned above
            int count = 0;
            for (int i = 0; i < 6; i++) {
                if(Core::RayCast::vsPlane(new Core::Ray(0.0f, 0.0f, 1.0f), planesOffsets[i], planesNormals[i], NULL))
                    count++;
            }*/
            
            //return true; //(count%2 == 1);
        }

        void Frostrum::updateMinMax(Core::Vector4 point)
        {
            if(point[0] < min[0]) min[0] = point[0];
            if(point[1] < min[1]) min[1] = point[1];
            if(point[2] < min[2]) min[2] = point[2];
            if(point[0] > max[0]) max[0] = point[0];
            if(point[1] > max[1]) max[1] = point[1];
            if(point[2] > max[2]) max[2] = point[2];
        }

        void Frostrum::updateFrostrum(const RenderData &data)
        {
            // Compute the 8 points of the frostrum from the RenderData and the max and min for the SAT algorithm
            Core::Matrix4 inverseView = data.viewMatrix.inverse();
            Core::Matrix4 inverseProj = data.projMatrix.inverse();

            m_a = inverseProj * inverseView * Core::Vector4(1.0f, 1.0f, 1.0f, 1.0f);
            std::cout << "A : " << std::endl << m_a << std::endl;
            max[0] = m_a[0];
            max[1] = m_a[1];
            max[2] = m_a[2];
            min[0] = m_a[0];
            min[1] = m_a[1];
            min[2] = m_a[2];
            m_b = inverseView * inverseProj *  Core::Vector4f(1.0f, 1.0f, -1.0f, 1.0f);
            std::cout << "B : " << std::endl << m_b << std::endl;
            updateMinMax(m_b);
            m_c = inverseView * inverseProj *  Core::Vector4(1.0f, -1.0f, 1.0f, 1.0f);
            std::cout << "C : " << std::endl << m_c << std::endl;
            updateMinMax(m_c);
            m_d = inverseView * inverseProj *  Core::Vector4(1.0f, -1.0f, -1.0f, 1.0f);
            std::cout << "D : " << std::endl << m_d << std::endl;
            updateMinMax(m_d);
            m_e = inverseView * inverseProj *  Core::Vector4(-1.0f, 1.0f, 1.0f, 1.0f);
            std::cout << "E : " << std::endl << m_e << std::endl;
            updateMinMax(m_e);
            m_f = inverseView * inverseProj *  Core::Vector4(-1.0f, 1.0f, -1.0f, 1.0f);
            std::cout << "F : " << std::endl << m_f << std::endl;
            updateMinMax(m_f);
            m_g = inverseView * inverseProj *  Core::Vector4(-1.0f, -1.0f, 1.0f, 1.0f);
            std::cout << "G : " << std::endl << m_g << std::endl;
            updateMinMax(m_g);
            m_h = inverseView * inverseProj *  Core::Vector4(-1.0f, -1.0f, -1.0f, 1.0f);
            std::cout << "H : " << std::endl << m_h << std::endl;
            updateMinMax(m_h);

            std::cout << "Min Frostrum : {" << min[0] << ", " << min[1] << ", " << min[2] << "}" << std::endl;
            std::cout << "Max Frostrum : {" << max[0] << ", " << max[1] << ", " << max[2] << "}" << std::endl;

            auto aabb = Core::Aabb(Core::Vector3(min[0], min[1], min[2]), Core::Vector3(max[0], max[1], max[2]));
            auto bb = Ra::Engine::DrawPrimitives::Primitive(Ra::Engine::SystemEntity::dbgCmp(), Ra::Engine::DrawPrimitives::AABB(aabb, Core::Colors::Magenta()));
            bb->setLifetime(1);
            Ra::Engine::SystemEntity::dbgCmp()->addRenderObject(bb);
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


