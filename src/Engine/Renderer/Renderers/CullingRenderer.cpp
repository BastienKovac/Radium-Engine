#include <Engine/Renderer/Renderers/CullingRenderer.hpp>

#include <Engine/Renderer/Renderers/ForwardRenderer.hpp>

#include <iostream>

#include <Core/Log/Log.hpp>
#include <Core/Math/ColorPresets.hpp>
#include <Core/Containers/Algorithm.hpp>
#include <Core/Containers/MakeShared.hpp>

#include <Engine/Renderer/RenderObject/RenderObject.hpp>
#include <Engine/RadiumEngine.hpp>
#include <Engine/Renderer/OpenGL/OpenGL.hpp>
#include <Engine/Renderer/RenderTechnique/RenderTechnique.hpp>
#include <Engine/Renderer/Material/Material.hpp>
#include <Engine/Renderer/RenderTechnique/ShaderProgramManager.hpp>
#include <Engine/Renderer/RenderTechnique/ShaderConfigFactory.hpp>

#include <Engine/System/System.hpp>
#include <Engine/Managers/SystemDisplay/SystemDisplay.hpp>

#include <Engine/Renderer/RenderTechnique/RenderParameters.hpp>
#include <Engine/Renderer/Light/Light.hpp>
#include <Engine/Renderer/Light/DirLight.hpp>
#include <Engine/Renderer/Light/DirLight.hpp>
#include <Engine/Renderer/Light/PointLight.hpp>
#include <Engine/Renderer/Light/SpotLight.hpp>
#include <Engine/Renderer/Mesh/Mesh.hpp>
#include <Engine/Renderer/Texture/TextureManager.hpp>
#include <Engine/Renderer/Texture/Texture.hpp>
#include <Engine/Renderer/Renderers/DebugRender.hpp>

#include <globjects/Framebuffer.h>
// Only for debug purpose, not needed here
#include <globjects/Program.h>

namespace Ra
{
    namespace Engine
    {

        namespace
        {
            const GLenum buffers[] =
            {
                GL_COLOR_ATTACHMENT0,
                GL_COLOR_ATTACHMENT1,
                GL_COLOR_ATTACHMENT2,
                GL_COLOR_ATTACHMENT3,
                GL_COLOR_ATTACHMENT4,
                GL_COLOR_ATTACHMENT5,
                GL_COLOR_ATTACHMENT6,
                GL_COLOR_ATTACHMENT7
            };
        }

        CullingRenderer::CullingRenderer()
            : ForwardRenderer()
        {
            m_cullingEnabled = true;
            m_cullingFixed = false;
        }

        CullingRenderer::~CullingRenderer()
        {
            ShaderProgramManager::destroyInstance();
        }

        void CullingRenderer::renderInternal(const RenderData &renderData)
        {
            const ShaderProgram *shader;

            m_fbo->bind();

            GL_ASSERT(glEnable(GL_DEPTH_TEST));
            GL_ASSERT(glDepthMask(GL_TRUE));
            GL_ASSERT(glColorMask(1, 1, 1, 1));

            GL_ASSERT(glDrawBuffers(4, buffers));

            const Core::Colorf clearColor = Core::Colors::FromChars<Core::Colorf>(42, 42, 42, 0);
            const Core::Colorf clearZeros = Core::Colors::Black<Core::Colorf>();
            const Core::Colorf clearOnes = Core::Colors::FromChars<Core::Colorf>(255, 255, 255, 255);
            const float clearDepth(1.0);

            GL_ASSERT(glClearBufferfv(GL_COLOR, 0, clearColor.data()));   // Clear color
            GL_ASSERT(glClearBufferfv(GL_COLOR, 1, clearZeros.data()));   // Clear normals
            GL_ASSERT(glClearBufferfv(GL_COLOR, 2, clearZeros.data()));   // Clear diffuse
            GL_ASSERT(glClearBufferfv(GL_COLOR, 3, clearZeros.data()));   // Clear specular
            GL_ASSERT(glClearBufferfv(GL_DEPTH, 0, &clearDepth));         // Clear depth

            // Z prepass

            GL_ASSERT(glDepthFunc(GL_LESS));
            GL_ASSERT(glDisable(GL_BLEND));

            GL_ASSERT(glPointSize(3.));

            if (m_cullingEnabled)
            {
                // Updates used camera in culling filter
                if (!m_cullingFixed)
                    m_cullingFilter.setFrostrum(renderData);

                std::cout << "Number of objects before culling : " << m_fancyRenderObjects.size() << std::endl;

                std::vector<RenderObjectPtr> filtered;
                for (const auto &ro : m_fancyRenderObjects)
                {
                    if (m_cullingFilter.intersectsFrostrum(ro->getAabb()))
                    {
                        filtered.push_back(ro);
                    }
                }
                m_fancyRenderObjects = filtered;
                std::cout << "Number of objects after culling : " << m_fancyRenderObjects.size() << std::endl;
            }

            // Set in RenderParam the configuration about ambiant lighting (instead of hard constant direclty in shaders)
            RenderParameters params;
            for (const auto &ro : m_fancyRenderObjects)
            {
             //   auto bb = Ra::Engine::DrawPrimitives::Primitive(Ra::Engine::SystemEntity::dbgCmp(), Ra::Engine::DrawPrimitives::AABB(ro->getAabb(), Core::Colors::Magenta()));
             //   bb->setLifetime(1);
             //   Ra::Engine::SystemEntity::dbgCmp()->addRenderObject(bb);
                ro->render(params, renderData, RenderTechnique::Z_PREPASS);
            }

            // Light pass
            GL_ASSERT(glDepthFunc(GL_LEQUAL));
            GL_ASSERT(glDepthMask(GL_FALSE));

            GL_ASSERT(glEnable(GL_BLEND));
            GL_ASSERT(glBlendFunc(GL_ONE, GL_ONE));

            GL_ASSERT(glDrawBuffers(1, buffers));   // Draw color texture

            if (m_lights.size() > 0)
            {
                for (const auto &l : m_lights)
                {
                    RenderParameters params;
                    l->getRenderParameters(params);

                    for (const auto &ro : m_fancyRenderObjects)
                    {
                        ro->render(params, renderData, RenderTechnique::LIGHTING_OPAQUE);
                    }
                }
            }
            else
            {
                DirectionalLight l;
                l.setDirection(Core::Vector3(0.3f, -1.0f, 0.0f));

                RenderParameters params;
                l.getRenderParameters(params);

                for (const auto &ro : m_fancyRenderObjects)
                {
                    ro->render(params, renderData, RenderTechnique::LIGHTING_OPAQUE);
                }
            }

#ifndef NO_TRANSPARENCY
            m_fbo->unbind();
            m_oitFbo->bind();

            GL_ASSERT(glDrawBuffers(2, buffers));
            GL_ASSERT(glClearBufferfv(GL_COLOR, 0, clearZeros.data()));
            GL_ASSERT(glClearBufferfv(GL_COLOR, 1, clearOnes.data()));

            GL_ASSERT(glDepthFunc(GL_LESS));
            GL_ASSERT(glEnable(GL_BLEND));

            GL_ASSERT(glBlendEquation(GL_FUNC_ADD));
            GL_ASSERT(glBlendFunci(0, GL_ONE, GL_ONE));
            GL_ASSERT(glBlendFunci(1, GL_ZERO, GL_ONE_MINUS_SRC_ALPHA));

            if (m_lights.size() > 0)
            {
                for (const auto &l : m_lights)
                {
                    RenderParameters params;
                    l->getRenderParameters(params);

                    for (const auto &ro : m_transparentRenderObjects)
                    {
                        ro->render(params, renderData, RenderTechnique::LIGHTING_TRANSPARENT);
                    }
                }
            }
            else
            {
                DirectionalLight l;
                l.setDirection(Core::Vector3(0.3f, -1.0f, 0.0f));

                RenderParameters params;
                l.getRenderParameters(params);

                for (const auto &ro : m_transparentRenderObjects)
                {
                    ro->render(params, renderData, RenderTechnique::LIGHTING_TRANSPARENT);
                }
            }

            m_oitFbo->unbind();

            m_fbo->bind();
            GL_ASSERT(glDrawBuffers(1, buffers));

            GL_ASSERT(glDepthFunc(GL_ALWAYS));
            GL_ASSERT(glBlendFunc(GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA));

            shader = m_shaderMgr->getShaderProgram("ComposeOIT");
            shader->bind();
            shader->setUniform("u_OITSumColor", m_textures[RendererTextures_OITAccum].get(), 0);
            shader->setUniform("u_OITSumWeight", m_textures[RendererTextures_OITRevealage].get(), 1);

            m_quadMesh->render();

#endif
            if (m_wireframe)
            {
                m_fbo->bind();

                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glEnable(GL_LINE_SMOOTH);
                glLineWidth(1.f);
                glEnable(GL_POLYGON_OFFSET_LINE);
                glPolygonOffset(-1.0f, -1.1f);

                // Light pass
                GL_ASSERT(glDepthFunc(GL_LEQUAL));
                GL_ASSERT(glDepthMask(GL_FALSE));

                GL_ASSERT(glEnable(GL_BLEND));
                GL_ASSERT(glBlendFunc(GL_ONE, GL_ONE));

                GL_ASSERT(glDrawBuffers(1, buffers));   // Draw color texture

                if (m_lights.size() > 0)
                {
                    for (const auto &l : m_lights)
                    {
                        RenderParameters params;
                        l->getRenderParameters(params);

                        for (const auto &ro : m_fancyRenderObjects)
                        {
                            ro->render(params, renderData, RenderTechnique::LIGHTING_OPAQUE);
                        }

                        for (size_t i = 0; i < m_fancyTransparentCount; ++i)
                        {
                            auto &ro = m_transparentRenderObjects[i];
                            ro->render(params, renderData, RenderTechnique::LIGHTING_OPAQUE);
                        }
                    }
                }
                else
                {
                    DirectionalLight l;
                    l.setDirection(Core::Vector3(0.3f, -1.0f, 0.0f));

                    RenderParameters params;
                    l.getRenderParameters(params);

                    for (const auto &ro : m_fancyRenderObjects)
                    {
                        ro->render(params, renderData, RenderTechnique::LIGHTING_OPAQUE);
                    }

                    for (size_t i = 0; i < m_fancyTransparentCount; ++i)
                    {
                        auto &ro = m_transparentRenderObjects[i];
                        ro->render(params, renderData, RenderTechnique::LIGHTING_OPAQUE);
                    }
                }

                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                glDisable(GL_POLYGON_OFFSET_LINE);

            }

            // Restore state
            GL_ASSERT(glDepthFunc(GL_LESS));
            GL_ASSERT(glDisable(GL_BLEND));
            GL_ASSERT(glDepthMask(GL_TRUE));

            m_fbo->unbind();
        }

    }
}
