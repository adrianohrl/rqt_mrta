#include <QVBoxLayout>
#include "rqt_mrta/define_architecture_widget.h"
#include "rqt_mrta/define_architecture_wizard_page.h"
#include "rqt_mrta/ui_define_architecture_widget.h"

namespace rqt_mrta
{
DefineArchitectureWizardPage::DefineArchitectureWizardPage(
    NewApplicationWizard* parent)
    : NewApplicationWizardPage(parent, "Define the Architecture")
{
  DefineArchitectureWidget* widget = new DefineArchitectureWidget(
      this, application_config_, architecture_config_);
  registerField("architecture*", widget->ui_->architectures_combo_box);
  connect(widget, SIGNAL(changed()), this,
          SIGNAL(completeChanged()));
  setWidget(widget);
}

DefineArchitectureWizardPage::~DefineArchitectureWizardPage() {}

bool DefineArchitectureWizardPage::validatePage()
{
  if (!application_config_ || !architecture_config_)
  {
    return false;
  }
  architecture_config_->load(application_config_->getApplicationPackageUrl() +
                             "/rqt_mrta.xml");
  return true;
}

bool DefineArchitectureWizardPage::isComplete() const
{
  return !application_config_->getApplication()
              ->getArchitecturePackage()
              .isEmpty();
}
}
