#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/application/robots.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/define_robots_parameters_widget.h"
#include "rqt_mrta/param_tree_widget.h"
#include "rqt_mrta/ui_define_robots_parameters_widget.h"

namespace rqt_mrta
{
DefineRobotsParametersWidget::DefineRobotsParametersWidget(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    RqtMrtaArchitectureConfig* architecture_config)
    : QWidget(parent), ui_(new Ui::DefineRobotsParametersWidget()),
      application_config_(NULL), architecture_config_(NULL)
{
  ui_->setupUi(this);
  ui_->parameters_tab_widget->clear();
  setArchitectureConfig(architecture_config);
  setApplicationConfig(application_config);
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

RqtMrtaApplicationConfig*
DefineRobotsParametersWidget::getApplicationConfig() const
{
  return application_config_;
}

void DefineRobotsParametersWidget::setArchitectureConfig(
    RqtMrtaArchitectureConfig* config)
{
  if (config != architecture_config_)
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

QString DefineRobotsParametersWidget::validate() const
{
  QString validation;
  for (size_t index(0); index < ui_->parameters_tab_widget->count(); index++)
  {
    ParamTreeWidget* tree = static_cast<ParamTreeWidget*>(
        ui_->parameters_tab_widget->widget(index));
    validation = tree->validate();
    if (!validation.isEmpty())
    {
      return validation;
    }
  }
  return validation;
}

void DefineRobotsParametersWidget::setApplicationConfig(
    RqtMrtaApplicationConfig* config)
{
  if (config != application_config_)
  {
    if (application_config_)
    {
      disconnect(application_config_, SIGNAL(changed()), this,
                 SLOT(applicationConfigChanged()));
      ui_->parameters_tab_widget->clear();
    }
    application_config_ = config;
    if (application_config_)
    {
      connect(application_config_, SIGNAL(changed()), this,
              SLOT(applicationConfigChanged()));
    }
  }
}

void DefineRobotsParametersWidget::loadTabs()
{
  ui_->parameters_tab_widget->clear();
  RobotsConfig* robots = application_config_->getApplication()->getRobots();
  config::Config* config = architecture_config_->getConfigs()->getConfig(
      architecture_config_->getArchitecture()->getRobots()->getConfigId());
  for (size_t index(0); index < robots->count(); index++)
  {
    RobotConfig* robot = robots->getRobot(index);
    robot->setConfig(config);
    ParamTreeWidget* tree = new ParamTreeWidget(ui_->parameters_tab_widget);
    tree->setConfig(robot->getConfig());
    ui_->parameters_tab_widget->addTab(tree, robot->getId());
  }
}

void DefineRobotsParametersWidget::architectureConfigChanged()
{
  emit changed();
}

void DefineRobotsParametersWidget::applicationConfigChanged()
{
  emit changed();
}
}
