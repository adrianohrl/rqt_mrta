#include <QVBoxLayout>
#include "rqt_mrta/new_application_wizard_page.h"

namespace rqt_mrta
{
NewApplicationWizardPage::NewApplicationWizardPage(NewApplicationWizard* parent,
                                                   const QString& title)
    : QWizardPage(parent), application_config_(parent->getApplicationConfig()),
      architecture_config_(parent->getArchitectureConfig()), setted_(false)
{
}

NewApplicationWizardPage::~NewApplicationWizardPage()
{
  architecture_config_ = NULL;
  application_config_ = NULL;
  if (widget_)
  {
    delete widget_;
    widget_ = NULL;
  }
}

void NewApplicationWizardPage::setWidget(QWidget* widget)
{
  if (!setted_)
  {
    widget_ = widget;
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(widget_);
    setLayout(layout);
    setted_ = true;
  }
}

void NewApplicationWizardPage::updateComplete() { emit completeChanged(); }
}
