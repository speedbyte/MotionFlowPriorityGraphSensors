#include "DisparityUpsamplingFilter.h"
#include "IPluginContainer.h"


class PluginContainer : public QObject, public IPluginContainer
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.Uni-Tue.Server.Plugins")
    Q_INTERFACES(IPluginContainer)
public:
    PluginContainer()
    {
		Add(IPluginDescriptionPtr(new DisparityUpsamplingFilterDescription()));
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





