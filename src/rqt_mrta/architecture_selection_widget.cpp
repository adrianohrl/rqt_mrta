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
  connect(ui_->robotsTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(filter()));
  connect(ui_->tasksTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(filter()));
  connect(ui_->allocationsTypeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(filter()));
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
      mrta_architectures_[package] = config_path;
    }
  }
}

void ArchitectureSelectionWidget::filter()
{
  ui_->architecturesComboBox->setEnabled(false);
  QStringList architectures(mrta_architectures_.keys());
  if (!architectures.isEmpty())
  {
    ui_->architecturesComboBox->addItems(architectures);
    ui_->architecturesComboBox->setEnabled(true);
  }
}
}
