#include <QFileDialog>
#include "rqt_mrta/select_architecture_config.h"
#include "rqt_mrta/mrta_widget.h"
#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/ui_mrta_widget.h"
#include "utilities/xml_settings.h"
#include <ros/package.h>
#include <ros/console.h>
#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"

namespace rqt_mrta
{
MRTAWidget::MRTAWidget(QWidget* parent)
    : QWidget(parent), ui_(new Ui::MRTAWidget()),
      architecture_config_(new RqtMrtaArchitectureConfig(this)),
      application_config_(new RqtMrtaApplicationConfig(this))
{
  ui_->setupUi(this);
  ui_->configuration_tab_widget->setCurrentIndex(0);
  ui_->architecture_tab->activateWindow();
  ui_->select_architecture_widget->setArchitectureConfig(architecture_config_);
  ui_->select_architecture_widget->setApplicationConfig(application_config_);
  ui_->new_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/new.png"))));
  ui_->open_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/open.png"))));
  ui_->launch_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/run.png"))));
  connect(ui_->new_application_push_button, SIGNAL(clicked()), this,
          SLOT(newPushButtonClicked()));
  connect(ui_->open_application_push_button, SIGNAL(clicked()), this,
          SLOT(openPushButtonClicked()));
}

MRTAWidget::~MRTAWidget()
{
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
  if (architecture_config_)
  {
    delete architecture_config_;
    architecture_config_ = NULL;
  }
  if (application_config_)
  {
    delete application_config_;
    application_config_ = NULL;
  }
}

bool MRTAWidget::loadConfig(const QString& url)
{
  if (architecture_config_ && !url.isEmpty())
  {
    QFileInfo file_info(url);
    if (file_info.isReadable())
    {
      QSettings settings(url, utilities::XmlSettings::format);
      if (settings.status() == QSettings::NoError)
      {
        architecture_config_->load(settings);
      }
    }
  }
}

void MRTAWidget::resetConfig()
{
  if (application_config_)
  {
    application_config_->reset();
  }
}

bool MRTAWidget::saveConfig()
{
  ROS_WARN_STREAM("[MRTAWidget] to aki");
  if (application_config_->getPackage().isEmpty())
  {
    return false;
  }
  std::string url(ros::package::getPath(application_config_->getPackage().toStdString()));
  ROS_WARN_STREAM("[MRTAWidget] pkg: " << url);
  if (url.empty())
  {
    return false;
  }
  url += "/rqt_mrta.xml";
  ROS_WARN_STREAM("[MRTAWidget] pkg: " << url);
  return saveConfig(QString::fromStdString(url));
}

bool MRTAWidget::saveConfig(const QString& url)
{
  if (application_config_ && !url.isEmpty())
  {
    QSettings settings(url, utilities::XmlSettings::format);
    if (settings.isWritable())
    {
      settings.clear();
      application_config_->save(settings);
      settings.sync();
      if (settings.status() == QSettings::NoError)
      {
        ROS_INFO_STREAM("Saved configuration to [" << url.toStdString() << "]");
        return true;
      }
    }
  }
}

bool MRTAWidget::createApplication()
{
  return true;
}

void MRTAWidget::newPushButtonClicked()
{
  NewApplicationWizard wizard;
  wizard.setWindowTitle("New Application");
  if (wizard.exec() == QWizard::Accepted)
  {
    ROS_INFO_STREAM("[MRTAWidget] accepted!!!");
    if (createApplication())
    {
      saveConfig();
    }
  }
}

void MRTAWidget::openPushButtonClicked()
{
  QFileDialog dialog(this, "Open Application Configuration", QDir::homePath(),
                     "MRTA configurations (*.xml)");
  dialog.setAcceptMode(QFileDialog::AcceptOpen);
  dialog.setFileMode(QFileDialog::ExistingFile);
  if (dialog.exec() == QDialog::Accepted)
  {
    ROS_INFO_STREAM("[MRTAWidget] openning: "
                    << dialog.selectedFiles().first().toStdString());
    loadConfig(dialog.selectedFiles().first());
  }
}
}
