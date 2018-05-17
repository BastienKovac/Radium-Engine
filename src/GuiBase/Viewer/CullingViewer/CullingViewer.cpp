#include <GuiBase/Viewer/CullingViewer/CullingViewer.hpp>

#include <iostream>

namespace Ra
{

    Gui::CullingViewer::CullingViewer(QScreen *screen)
        : Gui::Viewer(screen)
    {

    }

    Gui::CullingViewer::~CullingViewer()
    {

    }

    void Gui::CullingViewer::enableCulling(bool enabled)
    {
        std::cout << "enable" << std::endl;
        m_currentRenderer->enableCulling(enabled);
    }

    void Gui::CullingViewer::fixCulling(bool fixed)
    {
        std::cout << "fix" << std::endl;
        m_currentRenderer->fixCulling(fixed);
    }

}
