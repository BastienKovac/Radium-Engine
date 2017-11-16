#include <glbinding/Binding.h>
#include <glbinding/ContextInfo.h>
#include <glbinding/Version.h>
// Do not import namespace to prevent glbinding/QTOpenGL collision
#include <glbinding/gl/gl.h>

#include <globjects/globjects.h>

#include <Engine/RadiumEngine.hpp>

#include <GuiBase/Viewer/Viewer.hpp>

#include <iostream>

#include <QTimer>
#include <QMouseEvent>
#include <QPainter>

#include <Core/Containers/MakeShared.hpp>
#include <Core/Image/stb_image_write.h>
#include <Core/Log/Log.hpp>
#include <Core/Math/ColorPresets.hpp>
#include <Core/Math/Math.hpp>
#include <Core/String/StringUtils.hpp>

#include <Engine/Component/Component.hpp>

#include <Engine/Managers/SystemDisplay/SystemDisplay.hpp>
#include <Engine/Managers/EntityManager/EntityManager.hpp>

#include <Engine/Renderer/Camera/Camera.hpp>
#include <Engine/Renderer/Light/DirLight.hpp>
#include <Engine/Renderer/Renderer.hpp>
#include <Engine/Renderer/Renderers/ForwardRenderer.hpp>
#include <Engine/Renderer/RenderTechnique/ShaderProgramManager.hpp>

#include <GuiBase/Viewer/TrackballCamera.hpp>

#include <GuiBase/Utils/FeaturePickingManager.hpp>
#include <GuiBase/Utils/Keyboard.hpp>
#include <GuiBase/Utils/KeyMappingManager.hpp>

namespace Ra
{
    Gui::Viewer::Viewer( QWidget* parent )
        : QOpenGLWidget( parent )
        , m_currentRenderer( nullptr )
        , m_featurePickingManager( nullptr )
        , m_gizmoManager(new GizmoManager(this))
        , m_renderThread( nullptr )
        , m_glInitStatus( false )
        , m_hdpiScale( 1 )
    {
        // Allow Viewer to receive events
        setFocusPolicy( Qt::StrongFocus );

        setMinimumSize( QSize( 800, 600 ) );

        m_camera.reset( new Gui::TrackballCamera( width(), height() ) );
        m_featurePickingManager = new FeaturePickingManager();

        m_hdpiScale = devicePixelRatio();
        /// Intercept events to properly lock the renderer when it is compositing.
    }

    Gui::Viewer::~Viewer(){}


    int Gui::Viewer::addRenderer(std::shared_ptr<Engine::Renderer> e){
        CORE_ASSERT(m_glInitStatus.load(),
                    "OpenGL needs to be initialized to add renderers.");

        // initial state and lighting
        intializeRenderer(e.get());

        m_renderers.push_back(e);

        return m_renderers.size()-1;
    }

    void Gui::Viewer::initializeGL()
    {
        //no need to initalize glbinding. globjects (magically) do this internally.
        globjects::init(globjects::Shader::IncludeImplementation::Fallback);
        
        LOG( logINFO ) << "*** Radium Engine Viewer ***";
        LOG( logINFO ) << "Renderer (glbinding) : " << glbinding::ContextInfo::renderer();
        LOG( logINFO ) << "Vendor   (glbinding) : " << glbinding::ContextInfo::vendor();
        LOG( logINFO ) << "OpenGL   (glbinding) : " << glbinding::ContextInfo::version().toString();
        LOG( logINFO ) << "GLSL                 : " << gl::glGetString(gl::GLenum(GL_SHADING_LANGUAGE_VERSION));

        Engine::ShaderProgramManager::createInstance("Shaders/Default.vert.glsl",
                                                     "Shaders/Default.frag.glsl");

        auto light = Ra::Core::make_shared<Engine::DirectionalLight>();
        m_camera->attachLight( light );
            
        m_glInitStatus = true;
        emit glInitialized();

        if(m_renderers.empty()) {
            LOG(logWARNING)
                    << "Renderers fallback: no renderer added, enabling default (Forward Renderer)";
            std::shared_ptr<Ra::Engine::Renderer> e (new Ra::Engine::ForwardRenderer());
            addRenderer(e);
        }

        m_currentRenderer = m_renderers[0].get();

        emit rendererReady();
    }

    Gui::CameraInterface* Gui::Viewer::getCameraInterface()
    {
        return m_camera.get();
    }

    Gui::GizmoManager* Gui::Viewer::getGizmoManager()
    {
        return m_gizmoManager;
    }

    const Engine::Renderer* Gui::Viewer::getRenderer() const
    {
        return m_currentRenderer;
    }

    Engine::Renderer* Gui::Viewer::getRenderer()
    {
        return m_currentRenderer;
    }

    Gui::FeaturePickingManager* Gui::Viewer::getFeaturePickingManager()
    {
        return m_featurePickingManager;
    }

    void Gui::Viewer::onAboutToCompose()
    {
        // This slot function is called from the main thread as part of the event loop
        // when the GUI is about to update. We have to wait for the rendering to finish.
        m_currentRenderer->lockRendering();
    }

    void Gui::Viewer::onFrameSwapped()
    {
        // This slot is called from the main thread as part of the event loop when the
        // GUI has finished displaying the rendered image, so we unlock the renderer.
        m_currentRenderer->unlockRendering();
    }

    void Gui::Viewer::onAboutToResize()
    {
        // Like swap buffers, resizing is a blocking operation and we have to wait for the rendering
        // to finish before resizing.
        m_currentRenderer->lockRendering();
    }

    void Gui::Viewer::onResized()
    {
        m_currentRenderer->unlockRendering();
    }

    void Gui::Viewer::intializeRenderer(Engine::Renderer *renderer)
    {
        renderer->initialize(width(), height());
        if( m_camera->hasLightAttached() )
            renderer->addLight( m_camera->getLight() );
    }

    void Gui::Viewer::resizeGL( int width, int height )
    {
        // WARNING (Mathias) : on hdpi screens, this implies very big framebuffers
        m_hdpiScale = devicePixelRatio();
        // Renderer should have been locked by previous events.
        m_camera->resizeViewport( width*m_hdpiScale, height*m_hdpiScale );
        m_currentRenderer->resize( width*m_hdpiScale, height*m_hdpiScale );
    }

    void Gui::Viewer::mouseDoubleClickEvent( QMouseEvent* event )
    {
        if ( Gui::KeyMappingManager::getInstance()->actionTriggered( event, Gui::KeyMappingManager::VIEWER_BUTTON_SELECT_PICKING_QUERY ) )
        {
            // Check picking
            Engine::Renderer::PickingQuery query  = { Core::Vector2(event->x()*m_hdpiScale, (height() - event->y())*m_hdpiScale), Core::MouseButton::RA_MOUSE_RIGHT_BUTTON };
            m_currentRenderer->addPickingRequest(query);
        }
    }

    Engine::Renderer::PickingMode getPickingMode()
    {
        auto keyMap = Gui::KeyMappingManager::getInstance();
        if( Gui::isKeyPressed( keyMap->getKeyFromAction( Gui::KeyMappingManager::FEATUREPICKING_VERTEX ) ) )
        {
            return Engine::Renderer::VERTEX;
        }
        if( Gui::isKeyPressed( keyMap->getKeyFromAction( Gui::KeyMappingManager::FEATUREPICKING_EDGE ) ) )
        {
            return Engine::Renderer::EDGE;
        }
        if( Gui::isKeyPressed( keyMap->getKeyFromAction( Gui::KeyMappingManager::FEATUREPICKING_TRIANGLE ) ) )
        {
            return Engine::Renderer::TRIANGLE;
        }
        return Engine::Renderer::RO;
    }

    void Gui::Viewer::mousePressEvent( QMouseEvent* event )
    {
        auto keyMap = Gui::KeyMappingManager::getInstance();
        if( keyMap->actionTriggered( event, Gui::KeyMappingManager::VIEWER_BUTTON_MANIP_PICKING_QUERY ) )
        {
            if ( isKeyPressed( keyMap->getKeyFromAction(Gui::KeyMappingManager::VIEWER_RAYCAST_QUERY ) ) )
            {
                LOG( logINFO ) << "Raycast query launched";
                Core::Ray r = m_camera->getCamera()->getRayFromScreen(Core::Vector2(event->x()*m_hdpiScale, event->y()*m_hdpiScale));
                RA_DISPLAY_POINT(r.origin(), Core::Colors::Cyan(), 0.1f);
                RA_DISPLAY_RAY(r, Core::Colors::Yellow());
                auto ents = Engine::RadiumEngine::getInstance()->getEntityManager()->getEntities();
                for (auto e : ents)
                {
                    e->rayCastQuery(r);
                }
            }
            else
            {
                m_currentRenderer->addPickingRequest({ Core::Vector2(event->x()*m_hdpiScale, (height() - event->y())*m_hdpiScale),
                                                       Core::MouseButton::RA_MOUSE_LEFT_BUTTON,
                                                       Engine::Renderer::RO });
                m_gizmoManager->handleMousePressEvent(event, m_hdpiScale);
            }
        }
        else if ( keyMap->actionTriggered( event, Gui::KeyMappingManager::TRACKBALLCAMERA_MANIPULATION ) )
        {
            m_camera->handleMousePressEvent(event);
        }
        else if ( keyMap->actionTriggered( event, Gui::KeyMappingManager::VIEWER_BUTTON_SELECT_PICKING_QUERY ) )
        {
            // Check picking
            Engine::Renderer::PickingQuery query  = { Core::Vector2(event->x()*m_hdpiScale, (height() - event->y())*m_hdpiScale),
                                                      Core::MouseButton::RA_MOUSE_RIGHT_BUTTON,
                                                      getPickingMode() };
            m_currentRenderer->addPickingRequest(query);
        }
    }

    void Gui::Viewer::mouseReleaseEvent( QMouseEvent* event )
    {
        m_camera->handleMouseReleaseEvent( event );
        m_gizmoManager->handleMouseReleaseEvent(event);
    }

    void Gui::Viewer::mouseMoveEvent( QMouseEvent* event )
    {
        m_camera->handleMouseMoveEvent( event );
        m_gizmoManager->handleMouseMoveEvent(event, m_hdpiScale);
    }

    void Gui::Viewer::wheelEvent( QWheelEvent* event )
    {
        m_camera->handleWheelEvent(event);
        QOpenGLWidget::wheelEvent( event );
    }

    void Gui::Viewer::keyPressEvent( QKeyEvent* event )
    {
        keyPressed(event->key());
        m_camera->handleKeyPressEvent( event );

        QOpenGLWidget::keyPressEvent(event);
    }

    void Gui::Viewer::keyReleaseEvent( QKeyEvent* event )
    {
        keyReleased(event->key());
        m_camera->handleKeyReleaseEvent( event );

        if ( Gui::KeyMappingManager::getInstance()->actionTriggered( event, Gui::KeyMappingManager::VIEWER_TOGGLE_WIREFRAME ) && !event->isAutoRepeat())
        {
            m_currentRenderer->toggleWireframe();
        }

        QOpenGLWidget::keyReleaseEvent(event);
    }

    void Gui::Viewer::reloadShaders()
    {
        // FIXME : check thread-saefty of this.
        m_currentRenderer->lockRendering();
        makeCurrent();
        m_currentRenderer->reloadShaders();
        doneCurrent();
        m_currentRenderer->unlockRendering();
    }

    void Gui::Viewer::displayTexture( const QString &tex )
    {
        m_currentRenderer->lockRendering();
        m_currentRenderer->displayTexture( tex.toStdString() );
        m_currentRenderer->unlockRendering();
    }

    void Gui::Viewer::changeRenderer( int index )
    {
        if (m_renderers[index]) {
            if(m_currentRenderer != nullptr) m_currentRenderer->lockRendering();
            m_currentRenderer = m_renderers[index].get();
            m_currentRenderer->resize( width()*m_hdpiScale, height()*m_hdpiScale );
            m_currentRenderer->unlockRendering();
        }
    }

    // Asynchronous rendering implementation

    void Gui::Viewer::startRendering( const Scalar dt )
    {
        makeCurrent();

        // Move camera if needed. Disabled for now as it takes too long (see issue #69)
        //m_camera->update( dt );

        Engine::RenderData data;
        data.dt = dt;
        data.projMatrix = m_camera->getProjMatrix();
        data.viewMatrix = m_camera->getViewMatrix();

        m_currentRenderer->render( data );
    }

    void Gui::Viewer::waitForRendering()
    {
    }

    void Gui::Viewer::handleFileLoading( const std::string& file )
    {
        for ( auto& renderer : m_renderers )
        {
            if (renderer)
            {
                renderer->handleFileLoading( file );
            }
        }
    }

    void Gui::Viewer::handleFileLoading(const Ra::Asset::FileData &filedata) {
        for ( auto& renderer : m_renderers )
        {
            if (renderer)
            {
                renderer->handleFileLoading( filedata );
            }
        }
    }

    void Gui::Viewer::processPicking()
    {
        CORE_ASSERT( m_currentRenderer->getPickingQueries().size() == m_currentRenderer->getPickingResults().size(),
                     "There should be one result per query." );

        for (uint i = 0 ; i < m_currentRenderer->getPickingQueries().size(); ++i)
        {
            const Engine::Renderer::PickingQuery& query  = m_currentRenderer->getPickingQueries()[i];
            if ( query.m_button == Core::MouseButton::RA_MOUSE_LEFT_BUTTON)
            {
                emit leftClickPicking(m_currentRenderer->getPickingResults()[i]);
            }
            else if (query.m_button == Core::MouseButton::RA_MOUSE_RIGHT_BUTTON)
            {
                const int roIdx = m_currentRenderer->getPickingResults()[i];
                const Core::Ray ray = m_camera->getCamera()->getRayFromScreen({query.m_screenCoords(0), height()*m_hdpiScale-query.m_screenCoords(1)});
                // FIXME: this is safe as soon as there is no "queued connection" related to the signal
                m_featurePickingManager->doPicking(roIdx, query, ray);
                emit rightClickPicking(roIdx);
            }
        }
    }

    void Gui::Viewer::fitCameraToScene( const Core::Aabb& aabb )
    {
        if (!aabb.isEmpty())
        {
            m_camera->fitScene(aabb);
        }
        else
        {
            LOG( logINFO ) << "Unable to fit the camera to the scene : empty Bbox.";
        }
    }

    void Gui::Viewer::loadCamera(std::istream &in)
    {
        m_camera->load(in);
    }

    void Gui::Viewer::saveCamera(std::ostream& out) const
    {
        m_camera->save(out);
    }

    std::vector<std::string> Gui::Viewer::getRenderersName() const
    {
        std::vector<std::string> ret;

        for ( const auto& renderer : m_renderers )
        {
            if (renderer)
            {
                ret.push_back( renderer->getRendererName() );
            }
        }

        return ret;
    }

    void Gui::Viewer::grabFrame( const std::string& filename )
    {
        makeCurrent();

        uint w, h;
        uchar* writtenPixels = m_currentRenderer->grabFrame(w, h);

        std::string ext = Core::StringUtils::getFileExt(filename);

        if (ext == "bmp")
        {
            stbi_write_bmp(filename.c_str(), w, h, 4, writtenPixels);
        }
        else if (ext == "png")
        {
            stbi_write_png(filename.c_str(), w, h, 4, writtenPixels, w * 4 * sizeof(uchar));
        }
        else
        {
            LOG(logWARNING) << "Cannot write frame to "<<filename<<" : unsupported extension";
        }


        delete[] writtenPixels;

    }

    void Gui::Viewer::enablePostProcess(int enabled)
    {
        m_currentRenderer->enablePostProcess(enabled);
    }

    void Gui::Viewer::enableDebugDraw(int enabled)
    {
        m_currentRenderer->enableDebugDraw(enabled);
    }

    void Gui::Viewer::resetCamera()
    {
        m_camera.reset( new Gui::TrackballCamera( width()*m_hdpiScale, height()*m_hdpiScale ) );
    }

} // namespace Ra
