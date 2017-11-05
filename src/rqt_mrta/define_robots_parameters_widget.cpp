#include <QSettings>
#include <ros/console.h>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/define_robots_parameters_widget.h"
#include "rqt_mrta/ui_define_robots_parameters_widget.h"

namespace rqt_mrta
{
DefineRobotsParametersWidget::DefineRobotsParametersWidget(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    RqtMrtaArchitectureConfig* architecture_config)
    : QWidget(parent), ui_(new Ui::DefineRobotsParametersWidget()),
      application_config_(application_config),
      architecture_config_(architecture_config)
{
  ui_->setupUi(this);
}

DefineRobotsParametersWidget::~DefineRobotsParametersWidget()
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
DefineRobotsParametersWidget::getArchitectureConfig() const
{
  return architecture_config_;
}

RqtMrtaApplicationConfig* DefineRobotsParametersWidget::getApplicationConfig() const
{
  return application_config_;
}

void DefineRobotsParametersWidget::setArchitectureConfig(
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

void DefineRobotsParametersWidget::setApplicationConfig(
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

void DefineRobotsParametersWidget::architectureConfigChanged() { emit changed(); }

void DefineRobotsParametersWidget::applicationConfigChanged() { emit changed(); }
}
