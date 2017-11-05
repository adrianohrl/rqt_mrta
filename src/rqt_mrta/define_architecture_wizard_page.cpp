#include <QVBoxLayout>
#include "rqt_mrta/define_architecture_widget.h"
#include "rqt_mrta/define_architecture_wizard_page.h"
#include "rqt_mrta/ui_define_architecture_widget.h"

namespace rqt_mrta
{
DefineArchitectureWizardPage::DefineArchitectureWizardPage(NewApplicationWizard* parent)
    : QWizardPage(parent), id_(-1),
      application_config_(parent->getApplicationConfig()),
      architecture_config_(parent->getArchitectureConfig()),
      widget_(new DefineArchitectureWidget(this, application_config_,
                                           architecture_config_))
{
  setTitle("Define the Application Architecture");
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(widget_);
  setLayout(layout);
  registerField("name*", widget_->ui_->name_line_edit);
  registerField("package*", widget_->ui_->package_line_edit);
  registerField("architecture*", widget_->ui_->architectures_combo_box);
  connect(widget_->ui_->name_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(updateComplete()));
  connect(widget_->ui_->package_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(updateComplete()));
  connect(widget_->ui_->architectures_combo_box, SIGNAL(changed()), this,
          SLOT(updateComplete()));
}

DefineArchitectureWizardPage::~DefineArchitectureWizardPage()
{
  architecture_config_ = NULL;
  application_config_ = NULL;
  if (widget_)
  {
    delete widget_;
    widget_ = NULL;
  }
}

int DefineArchitectureWizardPage::getId() const { return id_; }

void DefineArchitectureWizardPage::setId(int id)
{
  if (id != id_)
  {
    id_ = id;
  }
}

void DefineArchitectureWizardPage::initializePage()
{
  application_config_->reset();
  architecture_config_->reset();
}

bool DefineArchitectureWizardPage::validatePage()
{
  widget_->loadConfig();
}

bool DefineArchitectureWizardPage::isComplete() const
{
  return !application_config_->getPackage().isEmpty() &&
         application_config_->getPackageUrl().isEmpty() &&
         !application_config_->getPackage().contains(' ') &&
         !application_config_->getApplication()->getName().isEmpty() &&
         !application_config_->getApplication()->getUrl().isEmpty();
}

void DefineArchitectureWizardPage::updateComplete() { emit completeChanged(); }
}
