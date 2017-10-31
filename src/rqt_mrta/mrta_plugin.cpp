#include <ros/package.h>
#include <rospack/macros.h>
#include <rospack/rospack.h>
#include "rqt_mrta/mrta_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <rqt_mrta/mrta_widget.h>

PLUGINLIB_DECLARE_CLASS(rqt_mrta, MRTAPlugin, rqt_mrta::MRTAPlugin,
                        rqt_gui_cpp::Plugin)

namespace rqt_mrta
{
MRTAPlugin::MRTAPlugin()
    : rqt_gui_cpp::Plugin(), mrta_widget_(NULL)
{
  setObjectName("MRTAPlugin");
}

MRTAPlugin::~MRTAPlugin()
{
}

void MRTAPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  mrta_widget_ = new MRTAWidget();
  context.addWidget(mrta_widget_);
}
}
