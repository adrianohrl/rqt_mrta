#ifndef _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
#define _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_

#include <QWidget>

namespace Ui
{
class ArchitectureSelectionWidget;
}

namespace mrta
{
class Architecture;
}

namespace rqt_mrta
{
class ArchitectureSelectionConfig;

class ArchitectureSelectionWidget : public QWidget
{
  Q_OBJECT
public:
  ArchitectureSelectionWidget(QWidget* parent);
  virtual ~ArchitectureSelectionWidget();
  ArchitectureSelectionConfig* getConfig() const;
  void setConfig(ArchitectureSelectionConfig* config);
  bool loadConfig(const QString& url);
  bool saveCurrentConfig();
  bool saveConfig(const QString& url);
  void resetConfig();

signals:
  void currentConfigModifiedChanged(bool modified);

private:
  typedef QList<mrta::Architecture*>::iterator iterator;
  typedef QList<mrta::Architecture*>::const_iterator const_iterator;
  Ui::ArchitectureSelectionWidget* ui_;
  ArchitectureSelectionConfig* config_;
  QList<mrta::Architecture*> architectures_;
  mrta::Architecture* selected_architecture_;
  bool current_config_modified_;
  void loadArchitectures();
  bool setCurrentConfigModified(bool modified);

private slots:
  void configChanged();
  void filter();
  void architectureChanged();
};
}

#endif // _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
