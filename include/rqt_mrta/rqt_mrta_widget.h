#ifndef _RQT_MRTA_MRTA_WIDGET_H_
#define _RQT_MRTA_MRTA_WIDGET_H_

#include <QWidget>

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
  RqtMrtaWidget(QWidget* parent = NULL);
  virtual ~RqtMrtaWidget();
  bool loadConfig(const QString& url);
  void resetConfig();
  bool saveConfig();
  bool saveConfig(const QString& url);

private:
  Ui::RqtMrtaWidget* ui_;
  RqtMrtaArchitectureConfig* architecture_config_;
  RqtMrtaApplicationConfig* application_config_;
  utilities::MessageSubscriberRegistry* registry_;

private slots:
  void newPushButtonClicked();
  void openPushButtonClicked();
};
}

#endif // _RQT_MRTA_MRTA_WIDGET_H_
