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
RqtMrtaWidget::RqtMrtaWidget(QWidget* parent,
                             const qt_gui_cpp::PluginContext& context)
    : QWidget(parent), ui_(new Ui::RqtMrtaWidget()),
      architecture_config_(new RqtMrtaArchitectureConfig(this)),
      application_config_(new RqtMrtaApplicationConfig(this)),
      registry_(new utilities::MessageSubscriberRegistry(this)),
      context_(context), loader_("rqt_gui", "rqt_gui_cpp::Plugin")
{
  ui_->setupUi(this);
  ui_->runtime_tab_widget->setCurrentIndex(0);
  ui_->architecture_tab->activateWindow();
  ui_->new_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/new.png"))));
  ui_->open_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/open.png"))));
  ui_->launch_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/run.png"))));
  ui_->new_architecture_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/new.png"))));
  ui_->open_architecture_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/open.png"))));
  connect(ui_->new_application_push_button, SIGNAL(clicked()), this,
          SLOT(newApplicationPushButtonClicked()));
  connect(ui_->open_application_push_button, SIGNAL(clicked()), this,
          SLOT(openApplicationPushButtonClicked()));
  connect(ui_->new_architecture_push_button, SIGNAL(clicked()), this,
          SLOT(newArchitecturePushButtonClicked()));
  connect(ui_->open_architecture_push_button, SIGNAL(clicked()), this,
          SLOT(openArchitecturePushButtonClicked()));

  /*architecture_config_->load(
      "/home/adrianohrl/ros_ws/mrta_ws/src/alliance/alliance/rqt_mrta.xml");
  loadArchitecturePlugins();*/
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

void RqtMrtaWidget::newApplicationPushButtonClicked()
{
  NewApplicationWizard wizard(this, application_config_, architecture_config_);
  if (wizard.exec() == QWizard::Accepted)
  {
    loadArchitecturePlugins();
    ROS_INFO_STREAM("[RqtMrtaWidget] accepted!!!");
  }
}

void RqtMrtaWidget::openApplicationPushButtonClicked()
{
  QFileDialog dialog(this, "Open Application Configuration", QDir::homePath(),
                     "MRTA configurations (*.xml)");
  dialog.setAcceptMode(QFileDialog::AcceptOpen);
  dialog.setFileMode(QFileDialog::ExistingFile);
  if (dialog.exec() == QDialog::Accepted)
  {
    application_config_->load(dialog.selectedFiles().first());
  }
}

void RqtMrtaWidget::newArchitecturePushButtonClicked() {}

void RqtMrtaWidget::openArchitecturePushButtonClicked()
{
  QFileDialog dialog(this, "Open Architecture Configuration", QDir::homePath(),
                     "MRTA configurations (*.xml)");
  dialog.setAcceptMode(QFileDialog::AcceptOpen);
  dialog.setFileMode(QFileDialog::ExistingFile);
  if (dialog.exec() == QDialog::Accepted)
  {
    architecture_config_->load(dialog.selectedFiles().first());
  }
}

void RqtMrtaWidget::loadArchitecturePlugins()
{
  for (size_t index(1); index < ui_->runtime_tab_widget->count(); index++)
  {
    ui_->runtime_tab_widget->removeTab(index);
  }
  external_plugins_.clear();
  for (size_t index(0); index < architecture_config_->getWidgets()->count();
       index++)
  {
    QString plugin_name(
        architecture_config_->getWidgets()->getWidget(index)->getPluginName());
    try
    {
      PluginPtr external_plugin(
          loader_.createInstance(plugin_name.toStdString()));
      external_plugin->initPlugin(context_);
      external_plugins_.push_back(external_plugin);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s",
                ex.what());
    }
  }
}
}
