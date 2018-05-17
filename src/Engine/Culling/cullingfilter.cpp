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

        Fustrum::Fustrum()
        {

        }
        bool Fustrum::intersects(const Core::Aabb aabb)
        {
            int out = 0;
            for(int i = 0 ; i < 6 ; i++)
            {
                out = 0;
                out += (( m_planes[i].dot(Core::Vector4(aabb.min()[0], aabb.min()[1], aabb.min()[2], 1.0f)) < 0.0 ) ? 1 : 0 );
                out += (( m_planes[i].dot(Core::Vector4(aabb.max()[1], aabb.min()[1], aabb.min()[2], 1.0f)) < 0.0 ) ? 1 : 0 );
                out += (( m_planes[i].dot(Core::Vector4(aabb.min()[0], aabb.max()[1], aabb.min()[2], 1.0f)) < 0.0 ) ? 1 : 0 );
                out += (( m_planes[i].dot(Core::Vector4(aabb.max()[0], aabb.max()[1], aabb.min()[2], 1.0f)) < 0.0 ) ? 1 : 0 );
                out += (( m_planes[i].dot(Core::Vector4(aabb.min()[0], aabb.min()[1], aabb.max()[2], 1.0f)) < 0.0 ) ? 1 : 0 );
                out += (( m_planes[i].dot(Core::Vector4(aabb.max()[0], aabb.min()[1], aabb.max()[2], 1.0f)) < 0.0 ) ? 1 : 0 );
                out += (( m_planes[i].dot(Core::Vector4(aabb.min()[0], aabb.max()[1], aabb.max()[2], 1.0f)) < 0.0 ) ? 1 : 0 );
                out += (( m_planes[i].dot(Core::Vector4(aabb.max()[0], aabb.max()[1], aabb.max()[2], 1.0f)) < 0.0 ) ? 1 : 0 );

                if(out == 8) return false;
            }

            out = 0; for (int i = 0 ; i < 8 ; i++) out += ((m_points[i][0] > aabb.max()[0]) ? 1 : 0); if (out == 8) return false;
            out = 0; for (int i = 0 ; i < 8 ; i++) out += ((m_points[i][0] < aabb.min()[0]) ? 1 : 0); if (out == 8) return false;
            out = 0; for (int i = 0 ; i < 8 ; i++) out += ((m_points[i][1] > aabb.max()[1]) ? 1 : 0); if (out == 8) return false;
            out = 0; for (int i = 0 ; i < 8 ; i++) out += ((m_points[i][1] < aabb.min()[1]) ? 1 : 0); if (out == 8) return false;
            out = 0; for (int i = 0 ; i < 8 ; i++) out += ((m_points[i][2] > aabb.max()[2]) ? 1 : 0); if (out == 8) return false;
            out = 0; for (int i = 0 ; i < 8 ; i++) out += ((m_points[i][2] < aabb.min()[2]) ? 1 : 0); if (out == 8) return false;

            return true;
        }

        void Fustrum::updateFustrum(const RenderData &data)
        {
            // Compute the 8 points of the frostrum from the RenderData
            Core::Matrix4 inverseView = data.viewMatrix.inverse();
            Core::Matrix4 inverseProj = data.projMatrix.inverse();

            // Points
            m_points[0] = inverseView * inverseProj * Core::Vector4(- 1.0f, - 1.0f, - 1.0f, 1.0f);
            m_points[1] = inverseView * inverseProj * Core::Vector4(- 1.0f, - 1.0f, 1.0f, 1.0f);
            m_points[2] = inverseView * inverseProj * Core::Vector4(- 1.0f, 1.0f, - 1.0f, 1.0f);
            m_points[3] = inverseView * inverseProj * Core::Vector4(- 1.0f, 1.0f, 1.0f, 1.0f);
            m_points[4] = inverseView * inverseProj * Core::Vector4(1.0f, - 1.0f, - 1.0f, 1.0f);
            m_points[5] = inverseView * inverseProj * Core::Vector4(1.0f, - 1.0f, 1.0f, 1.0f);
            m_points[6] = inverseView * inverseProj * Core::Vector4(1.0f, 1.0f, - 1.0f, 1.0f);
            m_points[7] = inverseView * inverseProj * Core::Vector4(1.0f, 1.0f, 1.0f, 1.0f);

            m_points[0] /= m_points[0][3];
            m_points[1] /= m_points[1][3];
            m_points[2] /= m_points[2][3];
            m_points[3] /= m_points[3][3];
            m_points[4] /= m_points[4][3];
            m_points[5] /= m_points[5][3];
            m_points[6] /= m_points[6][3];
            m_points[7] /= m_points[7][3];

            // Planes
            Core::Vector3 temp;
            temp = (m_points[1] - m_points[0]).head<3>().cross((m_points[2] - m_points[0]).head<3>());
            m_planes[0] = Core::Vector4(temp[0], temp[1], temp[2], 0.0f);
            m_planes[0][3] = - m_planes[0].dot(m_points[0]);

            temp = (m_points[5] - m_points[4]).head<3>().cross((m_points[0] - m_points[4]).head<3>());
            m_planes[1] = Core::Vector4(temp[0], temp[1], temp[2], 0.0f);
            m_planes[1][3] = - m_planes[1].dot(m_points[4]);

            temp = (m_points[7] - m_points[6]).head<3>().cross((m_points[4] - m_points[6]).head<3>());
            m_planes[2] = Core::Vector4(temp[0], temp[1], temp[2], 0.0f);
            m_planes[2][3] = - m_planes[2].dot(m_points[6]);

            temp = (m_points[3] - m_points[2]).head<3>().cross((m_points[6] - m_points[2]).head<3>());
            m_planes[3] = Core::Vector4(temp[0], temp[1], temp[2], 0.0f);
            m_planes[3][3] = - m_planes[3].dot(m_points[2]);

            temp = (m_points[2] - m_points[0]).head<3>().cross((m_points[4] - m_points[0]).head<3>());
            m_planes[4] = Core::Vector4(temp[0], temp[1], temp[2], 0.0f);
            m_planes[4][3] = - m_planes[4].dot(m_points[0]);

            temp = (m_points[3] - m_points[7]).head<3>().cross((m_points[5] - m_points[7]).head<3>());
            m_planes[5] = Core::Vector4(temp[0], temp[1], temp[2], 0.0f);
            m_planes[5][3] = - m_planes[5].dot(m_points[7]);

        }

        CullingFilter::CullingFilter()
        {

        }

        void CullingFilter::setFustrum(const RenderData &data)
        {
            m_fustrum.updateFustrum(data);
        }

        bool CullingFilter::intersectsFustrum(const Core::Aabb aabb)
        {
            return m_fustrum.intersects(aabb);
        }

    }
}


