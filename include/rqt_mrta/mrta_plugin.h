#ifndef _RQT_MRTA_PLUGIN_H_
#define _RQT_MRTA_PLUGIN_H_

#include <QWidget>
#include <rqt_gui_cpp/plugin.h>

namespace rqt_mrta
{
class MRTAWidget;

class MRTAPlugin
  : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
  MRTAPlugin();
  virtual ~MRTAPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

private:
  QWidget* mrta_widget_;
};
}

#endif  // _RQT_MRTA_PLUGIN_H_
