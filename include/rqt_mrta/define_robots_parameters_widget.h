#ifndef _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIDGET_H_
#define _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIDGET_H_

#include <QWidget>

namespace Ui
{
class DefineRobotsParametersWidget;
}

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robot;
class Robots;
class RqtMrtaApplication;
}

namespace architecture
{
class RqtMrtaArchitecture;
}
}

typedef config::application::Robot RobotConfig;
typedef config::application::Robots RobotsConfig;
typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;

class DefineRobotsParametersWidget : public QWidget
{
  friend class DefineRobotsParametersWizardPage;
  Q_OBJECT
public:
  DefineRobotsParametersWidget(
      QWidget* parent, RqtMrtaApplicationConfig* application_config = NULL,
      RqtMrtaArchitectureConfig* architecture_config = NULL);
  virtual ~DefineRobotsParametersWidget();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  RqtMrtaArchitectureConfig* getArchitectureConfig() const;
  void setApplicationConfig(RqtMrtaApplicationConfig* config);
  void setArchitectureConfig(RqtMrtaArchitectureConfig* config);
  QString validate() const;
  void loadTabs();

signals:
  void changed();

private:
  Ui::DefineRobotsParametersWidget* ui_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;

private slots:
  void architectureConfigChanged();
  void applicationConfigChanged();
};
}

#endif // _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIDGET_H_
