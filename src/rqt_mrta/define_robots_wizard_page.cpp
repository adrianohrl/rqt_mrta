#include <QVBoxLayout>
#include "rqt_mrta/define_robots_widget.h"
#include "rqt_mrta/define_robots_wizard_page.h"
#include "rqt_mrta/ui_define_robots_widget.h"

namespace rqt_mrta
{
DefineRobotsWizardPage::DefineRobotsWizardPage(NewApplicationWizard* parent)
    : NewApplicationWizardPage(parent,
                               "Define the Application Robots and Tasks")
{
  DefineRobotsWidget* widget =
      new DefineRobotsWidget(this, application_config_, architecture_config_);
  setWidget(widget);
}

DefineRobotsWizardPage::~DefineRobotsWizardPage() {}

void DefineRobotsWizardPage::initializePage() {}

void DefineRobotsWizardPage::cleanupPage() { architecture_config_->reset(); }

bool DefineRobotsWizardPage::validatePage() {}

bool DefineRobotsWizardPage::isComplete() const { return true; }
}
