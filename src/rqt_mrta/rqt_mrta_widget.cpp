#include <QFileDialog>
#include <ros/package.h>
#include <ros/console.h>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/rqt_mrta_widget.h"
#include "rqt_mrta/ui_rqt_mrta_widget.h"
#include "utilities/message_subscriber_registry.h"
#include "utilities/xml_settings.h"

namespace rqt_mrta
{
RqtMrtaWidget::RqtMrtaWidget(QWidget* parent)
    : QWidget(parent), ui_(new Ui::RqtMrtaWidget()),
      architecture_config_(new RqtMrtaArchitectureConfig(this)),
      application_config_(new RqtMrtaApplicationConfig(this)),
      registry_(new utilities::MessageSubscriberRegistry(this))
{
  ui_->setupUi(this);
  ui_->configuration_tab_widget->setCurrentIndex(0);
  ui_->architecture_tab->activateWindow();
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

RqtMrtaWidget::~RqtMrtaWidget()
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
  if (registry_)
  {
    delete registry_;
    registry_ = NULL;
  }
}

bool RqtMrtaWidget::loadConfig(const QString& url)
{
  ROS_INFO("[RqtMrtaWidget] loadConfig(url)");
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

void RqtMrtaWidget::resetConfig()
{
  if (application_config_)
  {
    application_config_->reset();
  }
  if (architecture_config_)
  {
    architecture_config_->reset();
  }
}

bool RqtMrtaWidget::saveConfig()
{
  if (application_config_ && application_config_->getPackage().isEmpty())
  {
    return false;
  }
  std::string url(
      ros::package::getPath(application_config_->getPackage().toStdString()));
  ROS_WARN_STREAM("[RqtMrtaWidget] pkg: " << url);
  if (url.empty())
  {
    return false;
  }
  url += "/rqt_mrta.xml";
  ROS_WARN_STREAM("[RqtMrtaWidget] pkg: " << url);
  return saveConfig(QString::fromStdString(url));
}

bool RqtMrtaWidget::saveConfig(const QString& url)
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

bool RqtMrtaWidget::createApplication() { return true; }

void RqtMrtaWidget::newPushButtonClicked()
{
  NewApplicationWizard wizard(this, application_config_, architecture_config_);
  if (wizard.exec() == QWizard::Accepted)
  {
    ROS_INFO_STREAM("[RqtMrtaWidget] accepted!!!");
    if (createApplication())
    {
      saveConfig();
    }
  }
}

void RqtMrtaWidget::openPushButtonClicked()
{
  QFileDialog dialog(this, "Open Application Configuration", QDir::homePath(),
                     "MRTA configurations (*.xml)");
  dialog.setAcceptMode(QFileDialog::AcceptOpen);
  dialog.setFileMode(QFileDialog::ExistingFile);
  if (dialog.exec() == QDialog::Accepted)
  {
    ROS_INFO_STREAM("[RqtMrtaWidget] openning: "
                    << dialog.selectedFiles().first().toStdString());
    loadConfig(dialog.selectedFiles().first());
  }
}
}
