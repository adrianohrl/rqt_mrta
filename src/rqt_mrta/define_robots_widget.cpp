#include <QSettings>
#include <ros/console.h>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/define_robots_widget.h"
#include "rqt_mrta/ui_define_robots_widget.h"

namespace rqt_mrta
{
DefineRobotsWidget::DefineRobotsWidget(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    RqtMrtaArchitectureConfig* architecture_config)
    : QWidget(parent), ui_(new Ui::DefineRobotsWidget()),
      application_config_(application_config),
      architecture_config_(architecture_config)
{
  ui_->setupUi(this);
}

DefineRobotsWidget::~DefineRobotsWidget()
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
DefineRobotsWidget::getArchitectureConfig() const
{
  return architecture_config_;
}

RqtMrtaApplicationConfig* DefineRobotsWidget::getApplicationConfig() const
{
  return application_config_;
}

void DefineRobotsWidget::setArchitectureConfig(
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
      connect(config, SIGNAL(changed()), this,
              SLOT(architectureConfigChanged()));
    }
  }
}

void DefineRobotsWidget::setApplicationConfig(
    RqtMrtaApplicationConfig* config)
{
  if (application_config_ != config)
  {
    if (application_config_)
    {
      disconnect(application_config_, SIGNAL(changed()), this,
                 SLOT(applicationConfigChanged()));
    }
    application_config_ = config;
    if (application_config_)
    {
      connect(config, SIGNAL(changed()), this,
              SLOT(applicationConfigChanged()));
    }
  }
}

void DefineRobotsWidget::architectureConfigChanged() { emit changed(); }

void DefineRobotsWidget::applicationConfigChanged() { emit changed(); }
}
