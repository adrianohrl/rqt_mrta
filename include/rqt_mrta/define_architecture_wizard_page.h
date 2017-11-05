#ifndef _RQT_MRTA_DEFINE_ARCHITECTURE_WIZARD_PAGE_H_
#define _RQT_MRTA_DEFINE_ARCHITECTURE_WIZARD_PAGE_H_

#include "rqt_mrta/new_application_wizard.h"

namespace rqt_mrta
{
class DefineArchitectureWidget;

class DefineArchitectureWizardPage : public QWizardPage
{
  Q_OBJECT
public:
  DefineArchitectureWizardPage(NewApplicationWizard* parent);
  virtual ~DefineArchitectureWizardPage();
  int getId() const;
  void setId(int id);
  void initializePage();
  bool validatePage();
  bool isComplete() const;

signals:
  void completeChanged();

private:
  int id_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;
  DefineArchitectureWidget* widget_;

private slots:
  void updateComplete();
};
}

#endif // _RQT_MRTA_DEFINE_ARCHITECTURE_WIZARD_PAGE_H_
