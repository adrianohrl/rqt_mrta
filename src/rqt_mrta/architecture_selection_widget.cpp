#include "mrta/architecture.h"
#include <rospack/rospack.h>
#include "rqt_mrta/architecture_selection_widget.h"
#include "rqt_mrta/ui_architecture_selection_widget.h"
#include "ros/console.h"

namespace rqt_mrta
{
ArchitectureSelectionWidget::ArchitectureSelectionWidget(QWidget* parent)
    : QWidget(parent), ui_(new Ui::ArchitectureSelectionWidget())
{
  loadMRTAArchitectures();
  ui_->setupUi(this);
  connect(ui_->robotsTypeComboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(filter()));
  connect(ui_->tasksTypeComboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(filter()));
  connect(ui_->allocationsTypeComboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(filter()));
  filter();
}

ArchitectureSelectionWidget::~ArchitectureSelectionWidget()
{
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}

void ArchitectureSelectionWidget::loadMRTAArchitectures()
{
  std::vector<std::string> architectures;
  rospack::Rospack rp;
  rp.setQuiet(true);
  std::vector<std::string> search_path;
  rp.getSearchPathFromEnv(search_path);
  rp.crawl(search_path, true);
  mrta_architectures_.clear();
  if (rp.plugins("rqt_mrta", "config", "", architectures))
  {
    for (std::vector<std::string>::iterator it(architectures.begin());
         it != architectures.end(); it++)
    {
      std::size_t index(it->find(' '));
      QString package(QString::fromStdString(it->substr(0, index)));
      QString config_path(QString::fromStdString(it->substr(index + 1)));
      mrta::Architecture* mrta_architecture =
          new mrta::Architecture(this, package, config_path);
      mrta_architectures_.push_back(mrta_architecture);
    }
  }
}

void ArchitectureSelectionWidget::filter()
{
  ui_->architecturesComboBox->setEnabled(false);
  ui_->architecturesComboBox->clear();
  ui_->architecturesComboBox->addItem("");
  mrta::Taxonomy::AllocationType allocation_type(
      mrta::Taxonomy::getAllocationType(
          ui_->allocationsTypeComboBox->currentText()));
  mrta::Taxonomy::RobotType robot_type(
      mrta::Taxonomy::getRobotType(ui_->robotsTypeComboBox->currentText()));
  mrta::Taxonomy::TaskType task_type(
      mrta::Taxonomy::getTaskType(ui_->tasksTypeComboBox->currentText()));
  for (const_iterator it(mrta_architectures_.begin());
       it != mrta_architectures_.end(); it++)
  {
    mrta::Architecture* mrta_architecture = *it;
    if (mrta_architecture->belongs(allocation_type, robot_type, task_type))
    {
      ui_->architecturesComboBox->addItem(mrta_architecture->toString());
    }
  }
  if (ui_->architecturesComboBox->count() > 0)
  {
    ui_->architecturesComboBox->setEnabled(true);
  }
}
}
