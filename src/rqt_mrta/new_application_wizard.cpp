#include <QPushButton>
#include <QVBoxLayout>
#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/define_application_wizard_page.h"
#include "rqt_mrta/define_architecture_wizard_page.h"
#include "rqt_mrta/define_robots_wizard_page.h"
#include "rqt_mrta/define_robots_parameters_wizard_page.h"
#include "rqt_mrta/new_application_wizard_page.h"
#include <ros/console.h>
#include <ros/package.h>
#include "utilities/exception.h"

namespace rqt_mrta
{
NewApplicationWizard::NewApplicationWizard(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    RqtMrtaArchitectureConfig* architecture_config, Qt::WindowFlags flags)
    : QWizard(parent, flags), application_config_(application_config),
      metapackage_config_(new RqtMrtaApplicationMetapackageConfig(this)),
      architecture_config_(architecture_config)
{
  if (!application_config_)
  {
    throw utilities::Exception(
        "The application configuration must not be null.");
  }
  if (!architecture_config_)
  {
    throw utilities::Exception(
        "The architecture configuration must not be null.");
  }
  setPage(DefineApplication, new DefineApplicationWizardPage(this));
  ROS_INFO_STREAM("[NewApplicationWizard] after DefineApplication ...");
  setPage(DefineArchitecture, new DefineArchitectureWizardPage(this));
  ROS_INFO_STREAM("[NewApplicationWizard] after DefineArchitecture ...");
  //setPage(DefineRobots, new DefineRobotsWizardPage(this));
  //ROS_INFO_STREAM("[NewApplicationWizard] after DefineRobots ...");
  //setPage(DefineRobotsParameters, new DefineRobotsParametersWizardPage(this));
  //ROS_INFO_STREAM("[NewApplicationWizard] after DefineRobotsParameters ...");
  setWindowTitle("New Application");
  connect(this, SIGNAL(accepted()), this, SLOT(generate()));
  connect(this, SIGNAL(rejected()), this, SLOT(resetConfig()));
}

NewApplicationWizard::~NewApplicationWizard()
{
  application_config_ = NULL;
  architecture_config_ = NULL;
}

RqtMrtaApplicationConfig* NewApplicationWizard::getApplicationConfig() const
{
  return application_config_;
}

RqtMrtaApplicationMetapackageConfig*
NewApplicationWizard::getMetapackageConfig() const
{
  return metapackage_config_;
}

RqtMrtaArchitectureConfig* NewApplicationWizard::getArchitectureConfig() const
{
  return architecture_config_;
}

void NewApplicationWizard::generate()
{
  if (metapackage_config_->createPackage())
  {
    application_config_->setPackage(metapackage_config_->getName());
    application_config_->getApplication()->setUrl(metapackage_config_->getUrl());
    ROS_WARN_STREAM("Created package ["
                    << application_config_->getPackage().toStdString() << "] @ ["
                    << application_config_->getPackageUrl().toStdString() << "].");
    application_config_->save();
  }
}

void NewApplicationWizard::resetConfig()
{
  application_config_->reset();
  metapackage_config_->reset();
  architecture_config_->reset();
}
}
