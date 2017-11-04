#include "mrta/architecture.h"
#include <QFileInfo>
#include <QSettings>
#include <ros/console.h>
#include <ros/package.h>
#include <rospack/rospack.h>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/select_architecture_widget.h"
#include "rqt_mrta/ui_select_architecture_widget.h"
#include "utilities/xml_settings.h"

namespace rqt_mrta
{
SelectArchitectureWidget::SelectArchitectureWidget(QWidget* parent)
    : QWidget(parent), ui_(new Ui::SelectArchitectureWidget()),
      architecture_config_(NULL), application_config_(NULL),
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

SelectArchitectureWidget::~SelectArchitectureWidget()
{
  architecture_config_ = NULL;
  application_config_ = NULL;
  selected_architecture_ = NULL;
  for (QList<mrta::Architecture*>::iterator it(architectures_.begin());
       it != architectures_.end(); it++)
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
  }
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}

RqtMrtaArchitectureConfig*
SelectArchitectureWidget::getArchitectureConfig() const
{
  return architecture_config_;
}

RqtMrtaApplicationConfig* SelectArchitectureWidget::getApplicationConfig() const
{
  return application_config_;
}

void SelectArchitectureWidget::setArchitectureConfig(
    RqtMrtaArchitectureConfig* config)
{
  ROS_WARN_STREAM("[SelectArchitectureWidget] setting architecture ...");
  if (architecture_config_ != config)
  {
    if (architecture_config_)
    {
      disconnect(architecture_config_, SIGNAL(changed()), this,
                 SLOT(architectureConfigChanged()));
    }
    architecture_config_ = config;
    if (config)
    {
      connect(config, SIGNAL(changed()), this,
              SLOT(architectureConfigChanged()));
    }
  }
}

void SelectArchitectureWidget::setApplicationConfig(
    RqtMrtaApplicationConfig* config)
{
  ROS_WARN_STREAM("[SelectArchitectureWidget] setting application ...");
  if (application_config_ != config)
  {
    if (application_config_)
    {
      disconnect(application_config_, SIGNAL(changed()), this,
                 SLOT(applicationConfigChanged()));
      disconnect(application_config_, SIGNAL(packageChanged(const QString&)),
                 this, SLOT(applicationPackageChanged(const QString&)));
    }
    application_config_ = config;
    if (config)
    {
      connect(config, SIGNAL(changed()), this,
              SLOT(applicationConfigChanged()));
      connect(application_config_, SIGNAL(packageChanged(const QString&)), this,
              SLOT(applicationPackageChanged(const QString&)));
    }
    applicationPackageChanged(application_config_->getPackage());
  }
}

bool SelectArchitectureWidget::loadConfig()
{
  return loadConfig(selected_architecture_->getConfigFilePath());
}

bool SelectArchitectureWidget::loadConfig(const QString& url)
{
  if (architecture_config_ && !url.isEmpty())
  {
    QFileInfo file_info(url);
    if (file_info.isReadable())
    {
      QSettings settings(url, utilities::XmlSettings::format);
      if (settings.status() == QSettings::NoError)
      {
        architecture_config_->load(settings);
      }
    }
  }
}

bool SelectArchitectureWidget::saveCurrentConfig()
{
  if (!application_config_)
  {
    return false;
  }
  std::string package(application_config_->getPackage().toStdString());
  if (package.empty())
  {
    return false;
  }
  ROS_WARN_STREAM("[SelectArchitectureWidget] pkg: " << package);
  return saveConfig(QString::fromStdString(ros::package::getPath(package)) + "rqt_mrta.xml");
}

bool SelectArchitectureWidget::saveConfig(const QString& url)
{
  if (application_config_ && !url.isEmpty())
  {
    QSettings settings(url, utilities::XmlSettings::format);
    if (settings.isWritable())
    {
      settings.clear();
      application_config_->save(settings);
      settings.sync();
      if (settings.status() == QSettings::NoError)
      {
        ROS_INFO_STREAM("Saved configuration to [" << url.toStdString() << "]");
        return true;
      }
    }
  }
}

void SelectArchitectureWidget::resetConfig()
{
  if (architecture_config_)
  {
    architecture_config_->reset();
  }
  if (application_config_)
  {
    application_config_->reset();
  }
}

void SelectArchitectureWidget::loadArchitectures()
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
      QString architecture_config_path(
          QString::fromStdString(it->substr(index + 1)));
      mrta::Architecture* mrta_architecture =
          new mrta::Architecture(this, package, architecture_config_path);
      architectures_.push_back(mrta_architecture);
    }
  }
}

void SelectArchitectureWidget::architectureConfigChanged()
{

}

void SelectArchitectureWidget::applicationConfigChanged()
{

}

void SelectArchitectureWidget::applicationPackageChanged(const QString& package)
{
  ui_->package_line_edit->setText(package);
}

void SelectArchitectureWidget::architectureChanged()
{
  ui_->taxonomy_label->setText("");
  QString package(ui_->architectures_combo_box->currentText());
  for (const_iterator it(architectures_.begin()); it != architectures_.end();
       it++)
  {
    if (**it == package)
    {
      selected_architecture_ = *it;
      ui_->taxonomy_label->setText(
          mrta::Taxonomy::toQString(*selected_architecture_));
      break;
    }
  }
}

void SelectArchitectureWidget::filter()
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
  int counter(0), index(-1);
  for (const_iterator it(architectures_.begin()); it != architectures_.end();
       it++)
  {
    mrta::Architecture* architecture = *it;
    if (architecture->belongs(allocation_type, robot_type, task_type))
    {
      ui_->architectures_combo_box->addItem(architecture->getPackage());
      counter++;
      if (selected_architecture_ && *architecture == *selected_architecture_)
      {
        index = counter;
      }
    }
  }
  if (ui_->architectures_combo_box->count() > 0)
  {
    ui_->architectures_combo_box->setEnabled(true);
    connect(ui_->architectures_combo_box, SIGNAL(currentIndexChanged(int)),
            this, SLOT(architectureChanged()));
  }
  if (index != -1)
  {
    ui_->architectures_combo_box->setCurrentIndex(index);
  }
  else
  {
    selected_architecture_ = NULL;
    ui_->taxonomy_label->setText("");
  }
}
}
