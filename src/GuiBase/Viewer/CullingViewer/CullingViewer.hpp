#ifndef CULLINGVIEWER_HPP
#define CULLINGVIEWER_HPP

#include <GuiBase/RaGuiBase.hpp>
#include <GuiBase/Viewer/Viewer.hpp>

#include <Engine/Renderer/Renderers/CullingRenderer.hpp>

namespace Ra
{
    namespace Gui
    {

        class RA_GUIBASE_API CullingViewer : public Viewer
        {

        public:

            explicit CullingViewer(QScreen * screen = nullptr);
            virtual ~CullingViewer();

            void enableCulling(bool enabled);
            void fixCulling(bool fixed);

        protected:

            Engine::CullingRenderer* m_currentRenderer;

        };

    }
}

#endif
