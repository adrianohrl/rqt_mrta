#ifndef _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
#define _RQT_MRTA_NEW_APPLICATION_DIALOG_H_

#include <QWizard>
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"

namespace Ui
{
class NewApplicationWizard;
}

namespace rqt_mrta
{
typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;

class DefineArchitectureWizardPage;
class DefineRobotsWizardPage;

class NewApplicationWizard : public QWizard
{
  Q_OBJECT
public:
  NewApplicationWizard(QWidget* parent,
                       RqtMrtaApplicationConfig* application_config,
                       RqtMrtaArchitectureConfig* architecture_config,
                       Qt::WindowFlags flags = 0);
  virtual ~NewApplicationWizard();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  RqtMrtaArchitectureConfig* getArchitectureConfig() const;

private:
  int past_id_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;
  DefineArchitectureWizardPage* define_architecture_;
  DefineRobotsWizardPage* define_robots_;

private slots:
  void idChanged(int id);
  void generatePackage();
  void resetConfig();
};
}

#endif // _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
