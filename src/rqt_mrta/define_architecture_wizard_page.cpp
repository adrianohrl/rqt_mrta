#include <QVBoxLayout>
#include "rqt_mrta/define_architecture_widget.h"
#include "rqt_mrta/define_architecture_wizard_page.h"
#include "rqt_mrta/ui_define_architecture_widget.h"

namespace rqt_mrta
{
DefineArchitectureWizardPage::DefineArchitectureWizardPage(
    NewApplicationWizard* parent)
    : NewApplicationWizardPage(parent, "Define the Application Architecture")
{
  DefineArchitectureWidget* widget = new DefineArchitectureWidget(
        this, parent->getApplicationConfig(),
        parent->getArchitectureConfig());
  registerField("name*", widget->ui_->name_line_edit);
  registerField("package*", widget->ui_->package_line_edit);
  registerField("architecture*", widget->ui_->architectures_combo_box);
  connect(widget->ui_->name_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(updateComplete()));
  connect(widget->ui_->package_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(updateComplete()));
  connect(widget->ui_->architectures_combo_box, SIGNAL(changed()), this,
          SLOT(updateComplete()));
  setWidget(widget);
}

DefineArchitectureWizardPage::~DefineArchitectureWizardPage() {}

void DefineArchitectureWizardPage::initializePage()
{
  application_config_->reset();
  architecture_config_->reset();
}

bool DefineArchitectureWizardPage::validatePage()
{
  static_cast<DefineArchitectureWidget*>(widget_)->loadConfig();
}

bool DefineArchitectureWizardPage::isComplete() const
{
  return !application_config_->getPackage().isEmpty() &&
         application_config_->getPackageUrl().isEmpty() &&
         !application_config_->getPackage().contains(' ') &&
         !application_config_->getApplication()->getName().isEmpty() &&
         !application_config_->getApplication()->getUrl().isEmpty();
}
}
