#include <QVBoxLayout>
#include "rqt_mrta/define_robots_widget.h"
#include "rqt_mrta/define_robots_wizard_page.h"
#include "rqt_mrta/ui_define_robots_widget.h"

namespace rqt_mrta
{
DefineRobotsWizardPage::DefineRobotsWizardPage(
    NewApplicationWizard* parent, QWizardPage* next)
    : QWizardPage(parent), id_(-1),
      application_config_(parent->getApplicationConfig()),
      architecture_config_(parent->getArchitectureConfig()),
      widget_(new DefineRobotsWidget(this, application_config_,
                                           architecture_config_)),
      next_(next)
{
  setTitle("Define the Application Robots and Tasks");
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(widget_);
  setLayout(layout);
}

DefineRobotsWizardPage::~DefineRobotsWizardPage()
{
  architecture_config_ = NULL;
  application_config_ = NULL;
  if (widget_)
  {
    delete widget_;
    widget_ = NULL;
  }
}

int DefineRobotsWizardPage::getId() const { return id_; }

void DefineRobotsWizardPage::setId(int id)
{
  if (id != id_)
  {
    id_ = id;
  }
}

void DefineRobotsWizardPage::initializePage()
{
  application_config_->reset();
  architecture_config_->reset();
}

void DefineRobotsWizardPage::cleanupPage()
{
  architecture_config_->reset();
}

bool DefineRobotsWizardPage::validatePage()
{
}

bool DefineRobotsWizardPage::isComplete() const
{
  return !application_config_->getPackage().isEmpty() &&
         application_config_->getPackageUrl().isEmpty() &&
         !application_config_->getPackage().contains(' ') &&
         !application_config_->getApplication()->getName().isEmpty() &&
         !application_config_->getApplication()->getUrl().isEmpty();
}

void DefineRobotsWizardPage::updateComplete() { emit completeChanged(); }
}
