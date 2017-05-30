#include "DisparityRefinementFilter.h"
#include "DisparityToDepthFilter.h"
#include "SGBMFilter.h"
#include "IPluginContainer.h"
namespace ImageStreaming {


class PluginContainer : public QObject, public IPluginContainer
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.Uni-Tue.Server.Plugins")
    Q_INTERFACES(IPluginContainer)
public:
    PluginContainer()
    {
        Add(IPluginDescriptionPtr (new DisparityRefinementFilterDescription()));
        Add(IPluginDescriptionPtr (new DisparityToDepthFilterDescription()));
        Add(IPluginDescriptionPtr (new SGBMFilterDescription()));
    }

    virtual ~PluginContainer(){}

    virtual PluginDictionary* Plugins()
    {
        return &m_plugindict;
    }

    void Add(IPluginDescriptionPtr plugin)
    {
        m_plugindict.insert(plugin->getPluginID(), plugin);
    }

protected:
    PluginDictionary m_plugindict;
private:

};
}
