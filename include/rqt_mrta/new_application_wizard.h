#ifndef _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
#define _RQT_MRTA_NEW_APPLICATION_DIALOG_H_

#include <QWizard>
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/application/rqt_mrta_metapackage.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"

namespace Ui
{
class NewApplicationWizard;
}

namespace rqt_mrta
{
typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::application::RqtMrtaApplicationMetapackage RqtMrtaApplicationMetapackageConfig;
typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;

class NewApplicationWizard : public QWizard
{
  Q_OBJECT
public:
  enum Page
  {
    DefineApplication,
    DefineArchitecture,
    DefineRobots,
    DefineRobotsParameters
  };
  NewApplicationWizard(QWidget* parent,
                       RqtMrtaApplicationConfig* application_config,
                       RqtMrtaArchitectureConfig* architecture_config,
                       Qt::WindowFlags flags = 0);
  virtual ~NewApplicationWizard();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  RqtMrtaApplicationMetapackageConfig* getMetapackageConfig() const;
  RqtMrtaArchitectureConfig* getArchitectureConfig() const;

private:
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaApplicationMetapackageConfig* metapackage_config_;
  RqtMrtaArchitectureConfig* architecture_config_;

private slots:
  void generate();
  void resetConfig();
};
}

#endif // _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
