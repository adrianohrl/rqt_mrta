#ifndef _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
#define _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_

#include <QMap>
#include <QWidget>

namespace Ui
{
class ArchitectureSelectionWidget;
}

namespace rqt_mrta
{
class ArchitectureSelectionWidget : public QWidget
{
Q_OBJECT
public:
  ArchitectureSelectionWidget(QWidget* parent);
  virtual ~ArchitectureSelectionWidget();

private:
  Ui::ArchitectureSelectionWidget* ui_;
  QMap<QString, QString> mrta_architectures_;
  void loadMRTAArchitectures();

private slots:
  void filter();
};
}

#endif // _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
