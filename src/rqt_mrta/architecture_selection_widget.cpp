#include "mrta/architecture.h"
#include <QFileInfo>
#include <QSettings>
#include <rospack/rospack.h>
#include "rqt_mrta/architecture_selection_config.h"
#include "rqt_mrta/architecture_selection_widget.h"
#include "rqt_mrta/ui_architecture_selection_widget.h"
#include "utilities/xml_settings.h"

namespace rqt_mrta
{
ArchitectureSelectionWidget::ArchitectureSelectionWidget(QWidget* parent)
    : QWidget(parent), ui_(new Ui::ArchitectureSelectionWidget()),
      config_(NULL), current_config_modified_(false),
      selected_architecture_(NULL)
{
  loadArchitectures();
  ui_->setupUi(this);
  connect(ui_->robots_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(filter()));
  connect(ui_->tasks_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(filter()));
  connect(ui_->allocations_type_combo_box, SIGNAL(currentIndexChanged(int)),
          this, SLOT(filter()));
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

ArchitectureSelectionConfig* ArchitectureSelectionWidget::getConfig() const
{
  return config_;
}

void ArchitectureSelectionWidget::setConfig(ArchitectureSelectionConfig* config)
{
  if (config_ != config)
  {
    if (config_)
    {
      disconnect(config_, SIGNAL(changed()), this, SLOT(configChanged()));
    }
    config_ = config;
    if (config)
    {
      connect(config, SIGNAL(changed()), this, SLOT(configChanged()));
    }
  }
}

bool ArchitectureSelectionWidget::loadConfig(const QString& url)
{
  if (config_ && !url.isEmpty())
  {
    QFileInfo file_info(url);
    if (file_info.isReadable())
    {
      QSettings settings(url, utilities::XmlSettings::format);
      if (settings.status() == QSettings::NoError)
      {
        settings.beginGroup("rqt_mrta");
        config_->load(settings);
        settings.endGroup();
      }
    }
  }
}

bool ArchitectureSelectionWidget::saveCurrentConfig() {}

bool ArchitectureSelectionWidget::saveConfig(const QString& url) {}

void ArchitectureSelectionWidget::resetConfig() {}

void ArchitectureSelectionWidget::loadArchitectures()
{
  std::vector<std::string> architectures;
  rospack::Rospack rp;
  rp.setQuiet(true);
  std::vector<std::string> search_path;
  rp.getSearchPathFromEnv(search_path);
  rp.crawl(search_path, true);
  architectures_.clear();
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
      architectures_.push_back(mrta_architecture);
    }
  }
}

void ArchitectureSelectionWidget::filter()
{
  disconnect(ui_->architectures_combo_box, SIGNAL(currentIndexChanged(int)),
             this, SLOT(architectureChanged()));
  ui_->architectures_combo_box->setEnabled(false);
  ui_->architectures_combo_box->clear();
  ui_->architectures_combo_box->addItem("");
  mrta::Taxonomy::AllocationType allocation_type(
      mrta::Taxonomy::getAllocationType(
          ui_->allocations_type_combo_box->currentText()));
  mrta::Taxonomy::RobotType robot_type(
      mrta::Taxonomy::getRobotType(ui_->robots_type_combo_box->currentText()));
  mrta::Taxonomy::TaskType task_type(
      mrta::Taxonomy::getTaskType(ui_->tasks_type_combo_box->currentText()));
  for (const_iterator it(architectures_.begin()); it != architectures_.end();
       it++)
  {
    mrta::Architecture* architecture = *it;
    if (architecture->belongs(allocation_type, robot_type, task_type))
    {
      ui_->architectures_combo_box->addItem(architecture->getPackage());
    }
  }
  if (ui_->architectures_combo_box->count() > 0)
  {
    ui_->architectures_combo_box->setEnabled(true);
    connect(ui_->architectures_combo_box, SIGNAL(currentIndexChanged(int)),
            this, SLOT(architectureChanged()));
  }
}

void ArchitectureSelectionWidget::configChanged()
{
  setCurrentConfigModified(true);
}

void ArchitectureSelectionWidget::architectureChanged()
{
  QString package(ui_->architectures_combo_box->currentText());
  for (const_iterator it(architectures_.begin()); it != architectures_.end();
       it++)
  {
    if (**it == package)
    {
      selected_architecture_ = *it;
      break;
    }
  }
  loadConfig(selected_architecture_->getConfigFilePath());
}

bool ArchitectureSelectionWidget::setCurrentConfigModified(bool modified)
{
  if (modified != current_config_modified_)
  {
    current_config_modified_ = modified;
    emit currentConfigModifiedChanged(modified);
  }
}
}
