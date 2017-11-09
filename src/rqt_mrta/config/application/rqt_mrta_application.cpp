#include <QFileInfo>
#include <ros/console.h>
#include <rospack/rospack.h>
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "utilities/exception.h"
#include "utilities/xml_settings.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
RqtMrtaApplication::RqtMrtaApplication(QObject* parent)
    : AbstractConfig(parent), application_(new Application(this))
{
  reset();
  rp_.setQuiet(true);
  std::vector<std::string> search_path;
  rp_.getSearchPathFromEnv(search_path);
  rp_.crawl(search_path, true);
  connect(application_, SIGNAL(changed()), this, SLOT(applicationChanged()));
}

RqtMrtaApplication::~RqtMrtaApplication()
{
  if (application_)
  {
    delete application_;
    application_ = NULL;
  }
}

QString RqtMrtaApplication::getPackage() const { return package_; }

QString RqtMrtaApplication::getPackageUrl() const { return package_url_; }

Application* RqtMrtaApplication::getApplication() const { return application_; }

void RqtMrtaApplication::setPackage(const QString& package)
{
  if (package != package_)
  {
    package_ = package;
    std::string url;
    rp_.find(package.toStdString(), url);
    package_url_ = QString::fromStdString(url);
    emit packageChanged(package);
    emit changed();
  }
}

void RqtMrtaApplication::save() const { save("rqt_mrta.xml"); }

void RqtMrtaApplication::save(const QString& filename) const
{
  if (package_url_.isEmpty() || filename.isEmpty())
  {
    ROS_ERROR("[RqtMrtaApplication] unable to save application rqt_mrta.xml file.");
    return;
  }
  QString url(package_url_ + "/" + filename);
  QSettings settings(url, utilities::XmlSettings::format);
  if (settings.isWritable())
  {
    settings.clear();
    save(settings);
    settings.sync();
    if (settings.status() == QSettings::NoError)
    {
      ROS_INFO_STREAM("Saved application configuration file ["
                      << url.toStdString() << "].");
    }
  }
}

void RqtMrtaApplication::save(QSettings& settings) const
{
  settings.beginGroup("rqt_mrta");
  settings.setValue("@format", "application");
  application_->save(settings);
  settings.endGroup();
}

void RqtMrtaApplication::load(const QString& url)
{
  QFileInfo file_info(url);
  if (!file_info.isReadable())
  {
    return;
  }
  QSettings settings(url, utilities::XmlSettings::format);
  if (settings.status() == QSettings::NoError)
  {
    load(settings);
    ROS_INFO_STREAM("Loaded application configuration file ["
                    << url.toStdString() << "].");
  }
}

void RqtMrtaApplication::load(QSettings& settings)
{
  settings.beginGroup("rqt_mrta");
  QString type(settings.value("@format").toString());
  if (type.isEmpty())
  {
    throw utilities::Exception("The <rqt_mrta> tag in the input xml file must "
                               "have an attribute named 'format'.");
  }
  if (type != "application")
  {
    throw utilities::Exception("The 'format' attribute of the <rqt_mrta> tag in "
                               "the input xml file must be valued as "
                               "'application' to be loaded as an application "
                               "configuration file.");
  }
  application_->load(settings);
  settings.endGroup();
}

void RqtMrtaApplication::reset()
{
  setPackage("");
  application_->reset();
}

void RqtMrtaApplication::write(QDataStream& stream) const
{
  application_->write(stream);
}

void RqtMrtaApplication::read(QDataStream& stream)
{
  application_->read(stream);
}

RqtMrtaApplication& RqtMrtaApplication::
operator=(const RqtMrtaApplication& config)
{
  setPackage(config.package_);
  *application_ = *config.application_;
  return *this;
}

void RqtMrtaApplication::applicationChanged() { emit changed(); }
}
}
}
