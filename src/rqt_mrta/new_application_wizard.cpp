#include <QPushButton>
#include <QVBoxLayout>
#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/select_architecture_widget.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include <ros/console.h>

namespace rqt_mrta
{
NewApplicationWizard::NewApplicationWizard(QWidget* parent,
                                           Qt::WindowFlags flags)
    : QWizard(parent, flags),
      architecture_widget_(new SelectArchitectureWidget(this))
{
  setWindowTitle("New Application");
  connect(this, SIGNAL(accepted()), this, SLOT(generatePackage()));
  connect(this, SIGNAL(rejected()), this, SLOT(resetConfig()));
}

NewApplicationWizard::~NewApplicationWizard()
{
  architecture_widget_->setApplicationConfig(NULL);
  architecture_widget_->setArchitectureConfig(NULL);
  if (architecture_widget_)
  {
    delete architecture_widget_;
    architecture_widget_ = NULL;
  }
}

void NewApplicationWizard::setConfig(
    RqtMrtaApplicationConfig* application_config,
    RqtMrtaArchitectureConfig* architecture_config)
{
  ROS_INFO("[NewApplicationWizard] ro aki");
  architecture_widget_->setApplicationConfig(application_config);
  architecture_widget_->setArchitectureConfig(architecture_config);
}

void NewApplicationWizard::createPages()
{
  addPage(createArchitecturePage());
}

QWizardPage* NewApplicationWizard::createArchitecturePage() const
{
  if (!architecture_widget_->getApplicationConfig() ||
      !architecture_widget_->getArchitectureConfig())
  {
    return NULL;
  }
  QWizardPage* page = new QWizardPage();
  page->setTitle("Select an Architecture");
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(architecture_widget_);
  page->setLayout(layout);
  return page;
}

void NewApplicationWizard::generatePackage()
{
  ROS_INFO("[NewApplicationWizard] generatePackage");
}

void NewApplicationWizard::resetConfig()
{
  if (architecture_widget_->getApplicationConfig())
  {
    architecture_widget_->getApplicationConfig()->reset();
  }
  if (architecture_widget_->getArchitectureConfig())
  {
    architecture_widget_->getArchitectureConfig()->reset();
  }
}
}
