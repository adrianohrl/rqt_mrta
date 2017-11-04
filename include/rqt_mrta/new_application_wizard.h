#ifndef _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
#define _RQT_MRTA_NEW_APPLICATION_DIALOG_H_

#include <QWizard>

namespace Ui
{
class NewApplicationWizard;
}

namespace rqt_mrta
{
class SelectArchitectureWidget;

class NewApplicationWizard : public QWizard
{
  Q_OBJECT
public:
  NewApplicationWizard(QWidget* parent = 0, Qt::WindowFlags flags = 0);
  virtual ~NewApplicationWizard();
  QWizardPage* createArchitecturePage() const;

private:
  SelectArchitectureWidget* architecture_widget_;
};
}

#endif // _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
