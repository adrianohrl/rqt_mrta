#include <QVBoxLayout>
#include "rqt_mrta/new_application_wizard_page.h"

namespace rqt_mrta
{
NewApplicationWizardPage::NewApplicationWizardPage(NewApplicationWizard* parent,
                                                   const QString& title)
    : QWizardPage(parent), application_config_(parent->getApplicationConfig()),
      metapackage_config_(parent->getMetapackageConfig()),
      architecture_config_(parent->getArchitectureConfig()), setted_(false)
{
  setTitle(title);
}

NewApplicationWizardPage::~NewApplicationWizardPage()
{
  architecture_config_ = NULL;
  application_config_ = NULL;
  metapackage_config_ = NULL;
}

void NewApplicationWizardPage::setWidget(QWidget* widget)
{
  if (!setted_)
  {
    widget_ = widget;
    widget_->setParent(this);
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(widget_);
    setLayout(layout);
    setted_ = true;
  }
}
}
