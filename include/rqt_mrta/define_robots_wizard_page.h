#ifndef _RQT_MRTA_DEFINE_ROBOTS_WIZARD_PAGE_H_
#define _RQT_MRTA_DEFINE_ROBOTS_WIZARD_PAGE_H_

#include "rqt_mrta/new_application_wizard.h"

namespace rqt_mrta
{
class DefineRobotsWidget;

class DefineRobotsWizardPage : public QWizardPage
{
  Q_OBJECT
public:
  DefineRobotsWizardPage(NewApplicationWizard* parent, QWizardPage* next = NULL);
  virtual ~DefineRobotsWizardPage();
  int getId() const;
  void setId(int id);
  void initializePage();
  void cleanupPage();
  bool validatePage();
  bool isComplete() const;

signals:
  void completeChanged();

private:
  int id_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;
  DefineRobotsWidget* widget_;
  QWizardPage* next_;

private slots:
  void updateComplete();
};
}

#endif // _RQT_MRTA_DEFINE_ROBOTS_WIZARD_PAGE_H_

