#include "rqt_mrta/architecture_selection_config.h"
#include "rqt_mrta/mrta_widget.h"
#include "rqt_mrta/ui_mrta_widget.h"

namespace rqt_mrta
{
MRTAWidget::MRTAWidget(QWidget *parent)
  : QWidget(parent), ui_(new Ui::MRTAWidget()), config_(new ArchitectureSelectionConfig(this))
{
  ui_->setupUi(this);
  ui_->configuration_tab_widget->setCurrentIndex(0);
  ui_->architecture_tab->activateWindow();
  ui_->architecture_selection_widget->setConfig(config_);
  ui_->allocated_tasks_widget->setConfig(config_->getAllocatedTasksConfig());
  ui_->busy_robots_widget->setConfig(config_->getBusyRobotsConfig());
  ui_->idle_robots_widget->setConfig(config_->getIdleRobotsConfig());
  ui_->incoming_tasks_widget->setConfig(config_->getIncomingTasksConfig());
  ui_->allocated_tasks_widget->setGroupBoxTitle("Allocated Tasks");
  ui_->busy_robots_widget->setGroupBoxTitle("Busy Robots");
  ui_->idle_robots_widget->setGroupBoxTitle("Idle Robots");
  ui_->incoming_tasks_widget->setGroupBoxTitle("Incoming Tasks");
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
