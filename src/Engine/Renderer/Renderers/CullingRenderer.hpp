#ifndef CULLINGRENDERER_HPP
#define CULLINGRENDERER_HPP


#include <Engine/Renderer/Renderers/ForwardRenderer.hpp>

#include <Engine/Culling/cullingfilter.hpp>
#include <string>

namespace Ra
{
    namespace Engine
    {

        class RA_ENGINE_API CullingRenderer : public ForwardRenderer
        {

        public:

            CullingRenderer();
            virtual ~CullingRenderer();

            inline void enableCulling(bool enabled)
            {
                m_cullingEnabled = enabled;
            }

            inline void fixCulling(bool fixed)
            {
                m_cullingFixed = fixed;
            }

            virtual std::string getRendererName() const override
            {
                return "Culling Renderer";
            }

        protected:

            void renderInternal(const RenderData &renderData) override;

            bool m_cullingEnabled;
            bool m_cullingFixed;

            CullingFilter m_cullingFilter;

        };

    }
}

#endif
