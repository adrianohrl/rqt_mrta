#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/select_architecture_widget.h"
#include <QPushButton>
#include <QVBoxLayout>

namespace rqt_mrta
{
NewApplicationWizard::NewApplicationWizard(QWidget* parent,
                                           Qt::WindowFlags flags)
    : QWizard(parent, flags),
      architecture_widget_(new SelectArchitectureWidget(this))
{
  addPage(createArchitecturePage());
}

NewApplicationWizard::~NewApplicationWizard()
{
  if (architecture_widget_)
  {
    delete architecture_widget_;
    architecture_widget_ = NULL;
  }
}

QWizardPage* NewApplicationWizard::createArchitecturePage() const
{
  QWizardPage* page = new QWizardPage();
  page->setTitle("Select an Architecture");
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(architecture_widget_);
  page->setLayout(layout);
  return page;
}
}
