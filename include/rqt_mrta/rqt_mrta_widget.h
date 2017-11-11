#ifndef _RQT_MRTA_MRTA_WIDGET_H_
#define _RQT_MRTA_MRTA_WIDGET_H_

#include <QWidget>
#include <pluginlib/class_loader.h>
#include <rqt_gui_cpp/plugin.h>

namespace utilities
{
class MessageSubscriberRegistry;
}

namespace Ui
{
class RqtMrtaWidget;
}

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplication;
}

namespace architecture
{
class RqtMrtaArchitecture;
}
}

typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;

class RqtMrtaWidget : public QWidget
{
  Q_OBJECT
public:
  RqtMrtaWidget(QWidget* parent, const qt_gui_cpp::PluginContext& context);
  virtual ~RqtMrtaWidget();
  bool loadConfig(const QString& url);
  void resetConfig();
  bool saveConfig();
  bool saveConfig(const QString& url);

private:
  typedef boost::shared_ptr<rqt_gui_cpp::Plugin> PluginPtr;
  typedef std::vector<PluginPtr> VectorPluginPtr;
  Ui::RqtMrtaWidget* ui_;
  RqtMrtaArchitectureConfig* architecture_config_;
  RqtMrtaApplicationConfig* application_config_;
  utilities::MessageSubscriberRegistry* registry_;
  qt_gui_cpp::PluginContext context_;
  VectorPluginPtr external_plugins_;
  pluginlib::ClassLoader<rqt_gui_cpp::Plugin> loader_;

private slots:
  void newApplicationPushButtonClicked();
  void openApplicationPushButtonClicked();
  void newArchitecturePushButtonClicked();
  void openArchitecturePushButtonClicked();
  void loadArchitecturePlugins();
};
}

#endif // _RQT_MRTA_MRTA_WIDGET_H_
