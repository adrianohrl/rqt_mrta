#ifndef _RQT_MRTA_NEW_APPLICATION_WIZARD_PAGE_H_
#define _RQT_MRTA_NEW_APPLICATION_WIZARD_PAGE_H_

#include "rqt_mrta/new_application_wizard.h"

namespace rqt_mrta
{
class NewApplicationWizardPage : public QWizardPage
{
  Q_OBJECT
public:
  NewApplicationWizardPage(NewApplicationWizard* parent, const QString &title);
  virtual ~NewApplicationWizardPage();

signals:
  void completeChanged();

protected:
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;
  QWidget* widget_;
  void setWidget(QWidget* widget);

protected slots:
  void updateComplete();

private:
  bool setted_;

};
}

#endif // _RQT_MRTA_NEW_APPLICATION_WIZARD_PAGE_H_
