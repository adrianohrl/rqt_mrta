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
      architecture_config_(NULL), application_config_(NULL)
{
  ui_->setupUi(this);
  connect(ui_->allocations_type_combo_box, SIGNAL(currentIndexChanged(int)),
          this, SLOT(setFilterAllocationType()));
  connect(ui_->robots_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(setFilterRobotType()));
  connect(ui_->tasks_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(setFilterTaskType()));
  connect(ui_->architectures_combo_box, SIGNAL(unknownAchitecture()), this,
          SLOT(unknownAchitecture()));
  connect(ui_->architectures_combo_box, SIGNAL(currentArchitectureChanged(mrta::Architecture*)), this,
          SLOT(currentArchitectureChanged(mrta::Architecture*)));
  connect(ui_->name_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(nameChanged(const QString&)));
  connect(ui_->package_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(packageChanged(const QString&)));
}

SelectArchitectureWidget::~SelectArchitectureWidget()
{
  architecture_config_ = NULL;
  application_config_ = NULL;
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
  if (architecture_config_ != config)
  {
    if (architecture_config_)
    {
      disconnect(architecture_config_, SIGNAL(changed()), this,
                 SLOT(architectureConfigChanged()));
    }
    architecture_config_ = config;
    if (architecture_config_)
    {
      ROS_ERROR("[SelectArchitectureWidget] setting architecture config");
      connect(config, SIGNAL(changed()), this,
              SLOT(architectureConfigChanged()));
    }
  }
}

void SelectArchitectureWidget::setApplicationConfig(
    RqtMrtaApplicationConfig* config)
{
  if (application_config_ != config)
  {
    if (application_config_)
    {
      disconnect(application_config_, SIGNAL(changed()), this,
                 SLOT(applicationConfigChanged()));
      disconnect(application_config_->getApplication(), SIGNAL(nameChanged(const QString&)), this,
                 SLOT(applicationConfigNameChanged(const QString&)));
      disconnect(application_config_, SIGNAL(packageChanged(const QString&)),
                 this, SLOT(applicationConfigPackageChanged(const QString&)));
    }
    application_config_ = config;
    if (application_config_)
    {
      ROS_ERROR("[SelectArchitectureWidget] setting application config");
      connect(config, SIGNAL(changed()), this,
              SLOT(applicationConfigChanged()));
      connect(application_config_->getApplication(), SIGNAL(nameChanged(const QString&)), this,
                 SLOT(applicationConfigNameChanged(const QString&)));
      connect(application_config_, SIGNAL(packageChanged(const QString&)), this,
              SLOT(applicationConfigPackageChanged(const QString&)));
      applicationConfigPackageChanged(application_config_->getPackage());
    }
  }
}

bool SelectArchitectureWidget::loadConfig()
{
  mrta::Architecture* architecture =
      ui_->architectures_combo_box->getCurrentArchitecture();
  return architecture ? loadConfig(architecture->getConfigFilePath()) : false;
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
  return saveConfig(QString::fromStdString(ros::package::getPath(package)) +
                    "rqt_mrta.xml");
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

void SelectArchitectureWidget::architectureConfigChanged()
{
  emit changed();
}

void SelectArchitectureWidget::applicationConfigChanged()
{
  emit changed();
}

void SelectArchitectureWidget::applicationConfigNameChanged(const QString &name)
{
  ui_->name_line_edit->setText(name);
}

void SelectArchitectureWidget::applicationConfigPackageChanged(
    const QString& package)
{
  ui_->package_line_edit->setText(package);
}

void SelectArchitectureWidget::setFilterAllocationType()
{
  mrta::Taxonomy::AllocationType allocation_type(
      mrta::Taxonomy::getAllocationType(
          ui_->allocations_type_combo_box->currentText()));
  ui_->architectures_combo_box->setFilterAllocationType(allocation_type);
}

void SelectArchitectureWidget::setFilterRobotType()
{
  mrta::Taxonomy::RobotType robot_type(
      mrta::Taxonomy::getRobotType(ui_->robots_type_combo_box->currentText()));
  ui_->architectures_combo_box->setFilterRobotType(robot_type);
}

void SelectArchitectureWidget::setFilterTaskType()
{
  mrta::Taxonomy::TaskType task_type(
      mrta::Taxonomy::getTaskType(ui_->tasks_type_combo_box->currentText()));
  ui_->architectures_combo_box->setFilterTaskType(task_type);
}

void SelectArchitectureWidget::unknownAchitecture()
{
  ui_->taxonomy_label->setText("");
}

void SelectArchitectureWidget::currentArchitectureChanged(
    mrta::Architecture* architecture)
{
  ui_->taxonomy_label->setText(
      architecture ? mrta::Taxonomy::toQString(*architecture) : "");
}

void SelectArchitectureWidget::nameChanged(const QString& name)
{
  if (application_config_)
  {
    application_config_->getApplication()->setName(name);
  }
}

void SelectArchitectureWidget::packageChanged(const QString& package)
{
  if (application_config_)
  {
    application_config_->setPackage(package);
  }
  ROS_ERROR("[SelectArchitectureWidget] change package status");
}
}
