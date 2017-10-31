#ifndef _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
#define _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_

#include <QList>
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
class ArchitectureSelectionWidget : public QWidget
{
Q_OBJECT
public:
  ArchitectureSelectionWidget(QWidget* parent);
  virtual ~ArchitectureSelectionWidget();

private:
  typedef QList<mrta::Architecture*>::iterator iterator;
  typedef QList<mrta::Architecture*>::const_iterator const_iterator;
  Ui::ArchitectureSelectionWidget* ui_;
  QList<mrta::Architecture*> mrta_architectures_;
  void loadMRTAArchitectures();

private slots:
  void filter();
};
}

#endif // _RQT_MRTA_ARCHITECTURE_SELECTION_WIDGET_H_
