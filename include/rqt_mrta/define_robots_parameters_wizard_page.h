#ifndef _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIZARD_PAGE_H_
#define _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIZARD_PAGE_H_

#include "rqt_mrta/new_application_wizard_page.h"

namespace rqt_mrta
{
class DefineRobotsParametersWidget;

class DefineRobotsParametersWizardPage : public NewApplicationWizardPage
{
  Q_OBJECT
public:
  DefineRobotsParametersWizardPage(NewApplicationWizard* parent);
  virtual ~DefineRobotsParametersWizardPage();
  void initializePage();
  void cleanupPage();
  bool validatePage();
  bool isComplete() const;
};
}

#endif // _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIZARD_PAGE_H_

