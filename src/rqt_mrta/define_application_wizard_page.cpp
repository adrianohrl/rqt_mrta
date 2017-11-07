#include <QVBoxLayout>
#include <rqt_mrta/config/application/rqt_mrta_metapackage.h>
#include "rqt_mrta/define_application_widget.h"
#include "rqt_mrta/define_application_wizard_page.h"
#include "rqt_mrta/ui_define_application_widget.h"

namespace rqt_mrta
{
DefineApplicationWizardPage::DefineApplicationWizardPage(
    NewApplicationWizard* parent)
    : NewApplicationWizardPage(parent, "Define the Application")
{
  DefineApplicationWidget* widget =
      new DefineApplicationWidget(this, parent->getApplicationConfig());
  registerField("name*", widget->ui_->name_line_edit);
  registerField("package*", widget->ui_->package_line_edit);
  registerField("workspace_url*", widget->ui_->workspace_package_line_edit);
  registerField("version*", widget->ui_->version_line_edit);
  registerField("description*", widget->ui_->description_plain_text_edit);
  registerField("maintainer*", widget->ui_->maintainer_line_edit);
  registerField("maintainer_email", widget->ui_->maintainer_email_line_edit);
  registerField("license*", widget->ui_->license_line_edit);
  registerField("run_depends", widget->ui_->run_depends_plain_text_edit);
  connect(widget->ui_->name_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(updateComplete()));
  connect(widget->ui_->package_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(updateComplete()));
  connect(widget->ui_->workspace_package_line_edit,
          SIGNAL(textChanged(const QString&)), this, SLOT(updateComplete()));
  connect(widget->ui_->version_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(updateComplete()));
  connect(widget->ui_->description_plain_text_edit, SIGNAL(textChanged()), this,
          SLOT(updateComplete()));
  connect(widget->ui_->maintainer_line_edit,
          SIGNAL(textChanged(const QString&)), this, SLOT(updateComplete()));
  connect(widget->ui_->maintainer_email_line_edit,
          SIGNAL(textChanged(const QString&)), this, SLOT(updateComplete()));
  connect(widget->ui_->license_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(updateComplete()));
  connect(widget->ui_->run_depends_plain_text_edit, SIGNAL(textChanged()), this,
          SLOT(updateComplete()));
  setWidget(widget);
}

DefineApplicationWizardPage::~DefineApplicationWizardPage() {}

void DefineApplicationWizardPage::initializePage()
{
  application_config_->reset();
  architecture_config_->reset();
}

bool DefineApplicationWizardPage::isComplete() const
{
  return static_cast<DefineApplicationWidget*>(widget_)
             ->metapackage_config_->isValid();
}
}
