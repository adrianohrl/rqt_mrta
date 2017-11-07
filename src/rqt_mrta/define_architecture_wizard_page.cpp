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
        this, parent->getApplicationConfig(),
        parent->getArchitectureConfig());

  registerField("architecture*", widget->ui_->architectures_combo_box);
  connect(widget->ui_->architectures_combo_box, SIGNAL(changed()), this,
          SLOT(updateComplete()));
  setWidget(widget);
}

DefineArchitectureWizardPage::~DefineArchitectureWizardPage() {}

bool DefineArchitectureWizardPage::validatePage()
{
  static_cast<DefineArchitectureWidget*>(widget_)->loadConfig();
}

bool DefineArchitectureWizardPage::isComplete() const
{
  return !application_config_->getApplication()->getUrl().isEmpty();
}
}
