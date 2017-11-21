#include <QVBoxLayout>
#include "rqt_mrta/define_robots_parameters_widget.h"
#include "rqt_mrta/define_robots_parameters_wizard_page.h"
#include "rqt_mrta/ui_define_robots_parameters_widget.h"

namespace rqt_mrta
{
DefineRobotsParametersWizardPage::DefineRobotsParametersWizardPage(
    NewApplicationWizard* parent)
    : NewApplicationWizardPage(parent,
                               "Define the Application Robots and Tasks")
{
  DefineRobotsParametersWidget* widget = new DefineRobotsParametersWidget(
      this, application_config_, architecture_config_);
  connect(application_config_, SIGNAL(changed()), this, SIGNAL(completeChanged()));
  setWidget(widget);
}

DefineRobotsParametersWizardPage::~DefineRobotsParametersWizardPage() {}

void DefineRobotsParametersWizardPage::initializePage()
{
  static_cast<DefineRobotsParametersWidget*>(widget_)->loadTabs();
}

void DefineRobotsParametersWizardPage::cleanupPage() {}

bool DefineRobotsParametersWizardPage::validatePage()
{
}

bool DefineRobotsParametersWizardPage::isComplete() const
{
  return static_cast<DefineRobotsParametersWidget*>(widget_)->validate().isEmpty();
}
}
