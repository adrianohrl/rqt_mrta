#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/application/robots.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/define_parameters_widget.h"
#include "rqt_mrta/param_tree_widget.h"
#include "rqt_mrta/ui_define_parameters_widget.h"

namespace rqt_mrta
{
DefineParametersWidget::DefineParametersWidget(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    RqtMrtaArchitectureConfig* architecture_config)
    : QWidget(parent), ui_(new Ui::DefineParametersWidget()),
      application_config_(NULL), architecture_config_(NULL)
{
  ui_->setupUi(this);
  ui_->parameters_tab_widget->clear();
  setArchitectureConfig(architecture_config);
  setApplicationConfig(application_config);
}

DefineParametersWidget::~DefineParametersWidget()
{
  architecture_config_ = NULL;
  application_config_ = NULL;
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}

RqtMrtaArchitectureConfig* DefineParametersWidget::getArchitectureConfig() const
{
  return architecture_config_;
}

RqtMrtaApplicationConfig* DefineParametersWidget::getApplicationConfig() const
{
  return application_config_;
}

void DefineParametersWidget::setArchitectureConfig(
    RqtMrtaArchitectureConfig* config)
{
  if (config != architecture_config_)
  {
    if (architecture_config_)
    {
      disconnect(architecture_config_, SIGNAL(changed()), this,
                 SIGNAL(changed()));
    }
    architecture_config_ = config;
    if (architecture_config_)
    {
      connect(config, SIGNAL(changed()), this, SIGNAL(changed()));
    }
  }
}

QString DefineParametersWidget::validate() const
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

void DefineParametersWidget::setApplicationConfig(
    RqtMrtaApplicationConfig* config)
{
  if (config != application_config_)
  {
    if (application_config_)
    {
      disconnect(application_config_, SIGNAL(changed()), this,
                 SIGNAL(changed()));
      ui_->parameters_tab_widget->clear();
    }
    application_config_ = config;
    if (application_config_)
    {
      connect(application_config_, SIGNAL(changed()), this, SIGNAL(changed()));
    }
  }
}

void DefineParametersWidget::loadTabs()
{
  ui_->parameters_tab_widget->clear();
  RobotsConfig* robots = application_config_->getApplication()->getRobots();
  config::Config* template_config = architecture_config_->getConfigs()->getConfig(
      architecture_config_->getArchitecture()->getRobots()->getConfigId());
  config::Configs* configs = application_config_->getConfigs();
  for (size_t index(0); index < robots->count(); index++)
  {
    config::Config* config = configs->addConfig();
    *config = *template_config;
    config->hideArrays();
    RobotConfig* robot = robots->getRobot(index);
    config->setId(robot->getId() + "_" + config->getId());
    ParamTreeWidget* tree = new ParamTreeWidget(ui_->parameters_tab_widget);
    tree->setConfig(config);
    ui_->parameters_tab_widget->addTab(tree, robot->getId());
  }
}
}
