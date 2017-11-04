#ifndef _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
#define _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_

#include <QWidget>

namespace Ui
{
class SelectArchitectureWidget;
}

namespace mrta
{
class Architecture;
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

class SelectArchitectureWidget : public QWidget
{
  Q_OBJECT
public:
  SelectArchitectureWidget(QWidget* parent);
  virtual ~SelectArchitectureWidget();
  RqtMrtaArchitectureConfig* getArchitectureConfig() const;
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  void setArchitectureConfig(RqtMrtaArchitectureConfig* config);
  void setApplicationConfig(RqtMrtaApplicationConfig* config);
  bool loadConfig();
  bool loadConfig(const QString& url);
  bool saveCurrentConfig();
  bool saveConfig(const QString& url);
  void resetConfig();

signals:
  void changed();

private:
  Ui::SelectArchitectureWidget* ui_;
  RqtMrtaArchitectureConfig* architecture_config_;
  RqtMrtaApplicationConfig* application_config_;

private slots:
  void architectureConfigChanged();
  void applicationConfigChanged();
  void applicationConfigNameChanged(const QString& name);
  void applicationConfigPackageChanged(const QString& package);
  void architectureChanged();
  void setFilterAllocationType();
  void setFilterRobotType();
  void setFilterTaskType();
  void unknownAchitecture();
  void currentArchitectureChanged(mrta::Architecture* architecture);
  void nameChanged(const QString& name);
  void packageChanged(const QString& package);
};
}

#endif // _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
