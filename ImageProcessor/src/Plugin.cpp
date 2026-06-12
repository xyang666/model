#include "Plugin.h"
#include "ImageProcessor.h"
#include <cstring>

namespace
{
    using ProcessorType = ImageProcessor::ImageProcessor;

    inline ProcessorType* toProcessor(void* ptr)
    {
        return static_cast<ProcessorType*>(ptr);
    }

    inline const ProcessorType* toProcessor(const void* ptr)
    {
        return static_cast<const ProcessorType*>(ptr);
    }
}

namespace ImageProcessor
{
    Plugin g_plugin;

    static IP_Config default_config()
    {
        IP_Config cfg;
        std::memset(&cfg, 0, sizeof(cfg));
        cfg.filterType = IP_FILTER_NONE;
        cfg.processNoiseSigmaX = 10.0;
        cfg.processNoiseSigmaY = 10.0;
        cfg.processNoiseSigmaZ = 10.0;
        cfg.minDetectPixelCount = 1.0;
        cfg.minClassPixelCount = 10.0;
        cfg.minIdentPixelCount = 50.0;
        cfg.averageAspectRatio = 4.0;
        cfg.detectionSAF = 1.0;
        cfg.classificationSAF = 4.0;
        cfg.identificationSAF = 6.4;
        cfg.coastTime = 5.0;
        cfg.targetRecognitionEnabled = false;
        cfg.reportsVelocity = true;
        return cfg;
    }

    Plugin::Plugin()
        : m_processor(nullptr)
        , m_initialized(false)
    {
        std::memset(&m_config, 0, sizeof(m_config));
    }

    Plugin::~Plugin()
    {
        Finalize();
    }

    bool Plugin::Initialize(const IP_Config* cfg)
    {
        if (m_initialized)
            return false;

        m_config = (cfg != nullptr) ? *cfg : default_config();

        m_processor = new ProcessorType();
        if (!m_processor)
            return false;

        if (!toProcessor(m_processor)->Initialize(&m_config))
        {
            delete toProcessor(m_processor);
            m_processor = nullptr;
            return false;
        }

        m_initialized = true;
        return true;
    }

    bool Plugin::Finalize()
    {
        if (!m_initialized)
            return false;

        delete toProcessor(m_processor);
        m_processor = nullptr;
        m_initialized = false;
        return true;
    }

    int Plugin::ProcessImage(const IP_ImageInput* input,
                              IP_ProcessOutput* output)
    {
        if (!m_initialized || !input || !output)
            return -1;

        return toProcessor(m_processor)->ProcessImage(input, output);
    }

    int Plugin::NoDetectUpdate(double simTime,
                                IP_TrackOutput* output)
    {
        if (!m_initialized || !output)
            return -1;

        return toProcessor(m_processor)->NoDetectUpdate(simTime, output);
    }

    int Plugin::Reset()
    {
        if (!m_initialized)
            return -1;
        return toProcessor(m_processor)->Reset();
    }

    int Plugin::GetFilterState() const
    {
        if (!m_initialized)
            return IP_FILTER_UNINITIALIZED;
        return toProcessor(m_processor)->GetFilterState();
    }

    int Plugin::SetConfig(const IP_Config* cfg)
    {
        if (!m_initialized || !cfg)
            return -1;

        m_config = *cfg;
        return toProcessor(m_processor)->SetConfig(&m_config);
    }

    int Plugin::GetConfig(IP_Config* cfg) const
    {
        if (!m_initialized || !cfg)
            return -1;

        return toProcessor(m_processor)->GetConfig(cfg);
    }

} // namespace ImageProcessor

extern "C"
{
    IP_EXPORT bool IP_Initialize(const IP_Config* cfg)
    {
        return ImageProcessor::g_plugin.Initialize(cfg);
    }

    IP_EXPORT bool IP_Finalize()
    {
        return ImageProcessor::g_plugin.Finalize();
    }

    IP_EXPORT int IP_ProcessImage(const IP_ImageInput* input,
                                   IP_ProcessOutput* output)
    {
        return ImageProcessor::g_plugin.ProcessImage(input, output);
    }

    IP_EXPORT int IP_NoDetectUpdate(double simTime,
                                     IP_TrackOutput* output)
    {
        return ImageProcessor::g_plugin.NoDetectUpdate(simTime, output);
    }

    IP_EXPORT int IP_Reset()
    {
        return ImageProcessor::g_plugin.Reset();
    }

    IP_EXPORT int IP_GetFilterState()
    {
        return ImageProcessor::g_plugin.GetFilterState();
    }

    IP_EXPORT int IP_SetConfig(const IP_Config* cfg)
    {
        return ImageProcessor::g_plugin.SetConfig(cfg);
    }

    IP_EXPORT int IP_GetConfig(IP_Config* cfg)
    {
        return ImageProcessor::g_plugin.GetConfig(cfg);
    }
}
