#ifndef _RQT_MRTA_MRTA_WIDGET_H_
#define _RQT_MRTA_MRTA_WIDGET_H_

#include <QWidget>

namespace Ui
{
class MRTAWidget;
}

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class RqtMrtaArchitecture;
}

namespace application
{
class RqtMrtaApplication;
}
}

typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;
typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;

class MRTAWidget : public QWidget
{
  Q_OBJECT
public:
  MRTAWidget(QWidget* parent = NULL);
  virtual ~MRTAWidget();
  bool loadConfig(const QString& url);
  void resetConfig();
  bool saveConfig();
  bool saveConfig(const QString& url);
  bool createApplication();

private:
  Ui::MRTAWidget* ui_;
  RqtMrtaArchitectureConfig* architecture_config_;
  RqtMrtaApplicationConfig* application_config_;

private slots:
  void newPushButtonClicked();
  void openPushButtonClicked();
};
}

#endif // _RQT_MRTA_MRTA_WIDGET_H_
