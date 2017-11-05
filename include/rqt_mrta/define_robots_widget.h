#ifndef _RQT_MRTA_DEFINE_ROBOTS_WIDGET_H_
#define _RQT_MRTA_DEFINE_ROBOTS_WIDGET_H_

#include <QWidget>

namespace Ui
{
class DefineRobotsWidget;
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

class DefineRobotsWidget : public QWidget
{
  friend class DefineRobotsWizardPage;
  Q_OBJECT
public:
  DefineRobotsWidget(
      QWidget* parent, RqtMrtaApplicationConfig* application_config = NULL,
      RqtMrtaArchitectureConfig* architecture_config = NULL);
  virtual ~DefineRobotsWidget();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  RqtMrtaArchitectureConfig* getArchitectureConfig() const;
  void setApplicationConfig(RqtMrtaApplicationConfig* config);
  void setArchitectureConfig(RqtMrtaArchitectureConfig* config);

signals:
  void changed();

private:
  Ui::DefineRobotsWidget* ui_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;

private slots:
  void architectureConfigChanged();
  void applicationConfigChanged();
};
}

#endif // _RQT_MRTA_DEFINE_ROBOTS_WIDGET_H_
