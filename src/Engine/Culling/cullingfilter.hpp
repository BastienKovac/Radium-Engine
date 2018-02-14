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
            Frostrum(const RenderData &data);

            bool intersects(const std::shared_ptr<RenderObject> object);

        private:

            void updateCoordinates(const RenderData &data);

            // Points of the Frostrum
            Core::Vector3 m_a, m_b, m_c, m_d, m_e, m_f, m_g, m_h;

        };

        class CullingFilter
        {
        public:
            CullingFilter();

            // std::shared_ptr<RenderObject> defined as RenderObjectPtr (protected) in Renderer.hpp => Make it public ?
            bool intersectsFrostrums(const std::shared_ptr<RenderObject> object);

            void addFrostrum(const RenderData &data);

            void reset();

            // TODO : Maybe index is not the best way ?
            void removeFrostrum(int index);

        private:

            std::vector<Frostrum> m_frostrum;
        };

    } // namespace Engine
} // namespace Ra

#endif // CULLINGFILTER_H
