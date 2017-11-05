#include <QPushButton>
#include <QVBoxLayout>
#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/define_architecture_wizard_page.h"
#include "rqt_mrta/define_robots_wizard_page.h"
#include "rqt_mrta/define_robots_parameters_wizard_page.h"
#include "rqt_mrta/new_application_wizard_page.h"
#include <ros/console.h>
#include "utilities/exception.h"

namespace rqt_mrta
{
NewApplicationWizard::NewApplicationWizard(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    RqtMrtaArchitectureConfig* architecture_config, Qt::WindowFlags flags)
    : QWizard(parent, flags), application_config_(application_config),
      architecture_config_(architecture_config), past_id_(-1)
{
  if (!application_config_)
  {
    throw utilities::Exception("The application configuration must not be null.");
  }
  if (!architecture_config_)
  {
    throw utilities::Exception("The architecture configuration must not be null.");
  }
  setPage(DEFINE_ARCHITECTURE, new DefineArchitectureWizardPage(this));
  setPage(DEFINE_ROBOTS, new DefineRobotsWizardPage(this));
  setPage(DEFINE_ROBOTS_PARAMETERS, new DefineRobotsParametersWizardPage(this));
  setWindowTitle("New Application");
  connect(this, SIGNAL(accepted()), this, SLOT(generatePackage()));
  connect(this, SIGNAL(rejected()), this, SLOT(resetConfig()));
  connect(this, SIGNAL(currentIdChanged(int)), this, SLOT(idChanged(int)));
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

RqtMrtaArchitectureConfig* NewApplicationWizard::getArchitectureConfig() const
{
  return architecture_config_;
}

void NewApplicationWizard::idChanged(int id)
{
  bool moved_foward(id - past_id_ > 0);
  /*if (id == define_architecture_->getId())
  {
    architecture_config_->reset();
  }
  if (moved_foward)
  {
    if (id == define_architecture_->getId() + 1)
    {
      define_architecture_->loadArchitectureConfig();
    }
  }*/
  past_id_ = id;
}

void NewApplicationWizard::generatePackage()
{
  ROS_INFO("[NewApplicationWizard] generatePackage");
}

void NewApplicationWizard::resetConfig()
{
  if (application_config_)
  {
    application_config_->reset();
  }
  if (architecture_config_)
  {
    architecture_config_->reset();
  }
}
}
