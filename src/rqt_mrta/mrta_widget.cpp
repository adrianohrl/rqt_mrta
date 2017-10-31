#include "rqt_mrta/architecture_selection_widget.h"
#include "rqt_mrta/mrta_widget.h"
#include "rqt_mrta/ui_mrta_widget.h"

namespace rqt_mrta
{
MRTAWidget::MRTAWidget(QWidget *parent)
  : QWidget(parent), ui_(new Ui::MRTAWidget())
{
  ui_->setupUi(this);
}

MRTAWidget::~MRTAWidget()
{
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}
}
