#ifndef _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
#define _RQT_MRTA_NEW_APPLICATION_DIALOG_H_

#include <QWizard>

namespace Ui
{
class NewApplicationWizard;
}

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplication;
}

namespace architecture
{
class RqtMrtaArchitecture;
}
}

typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;

class SelectArchitectureWidget;

class NewApplicationWizard : public QWizard
{
  Q_OBJECT
public:
  NewApplicationWizard(QWidget* parent = 0, Qt::WindowFlags flags = 0);
  virtual ~NewApplicationWizard();
  void setConfig(RqtMrtaApplicationConfig* application_config,
                 RqtMrtaArchitectureConfig* architecture_config);
  void createPages();
  QWizardPage* createArchitecturePage() const;

private:
  SelectArchitectureWidget* architecture_widget_;

private slots:
  void generatePackage();
  void resetConfig();
};
}

#endif // _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
