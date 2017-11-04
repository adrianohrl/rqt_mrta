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

private:
  typedef QList<mrta::Architecture*>::iterator iterator;
  typedef QList<mrta::Architecture*>::const_iterator const_iterator;
  Ui::SelectArchitectureWidget* ui_;
  RqtMrtaArchitectureConfig* architecture_config_;
  RqtMrtaApplicationConfig* application_config_;
  QList<mrta::Architecture*> architectures_;
  mrta::Architecture* selected_architecture_;
  void loadArchitectures();

private slots:
  void architectureConfigChanged();
  void applicationConfigChanged();
  void applicationPackageChanged(const QString& package);
  void architectureChanged();
  void filter();
};
}

#endif // _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
