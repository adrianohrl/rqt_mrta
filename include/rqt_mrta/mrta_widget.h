#ifndef _RQT_MRTA_MRTA_WIDGET_H_
#define _RQT_MRTA_MRTA_WIDGET_H_

#include <QWidget>

namespace Ui
{
  class MRTAWidget;
}

namespace rqt_mrta
{
class ArchitectureSelectionWidget;

class MRTAWidget : public QWidget
{
Q_OBJECT
public:
  MRTAWidget(QWidget* parent = NULL);
  virtual ~MRTAWidget();

private:
  Ui::MRTAWidget* ui_;
};
}

#endif // _RQT_MRTA_MRTA_WIDGET_H_
