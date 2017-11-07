#include <rospack/rospack.h>
#include "rqt_mrta/config/application/rqt_mrta_application.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
RqtMrtaApplication::RqtMrtaApplication(QObject *parent)
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

QString RqtMrtaApplication::getPackage() const
{
  return package_;
}

QString RqtMrtaApplication::getPackageUrl() const
{
  return package_url_;
}

Application *RqtMrtaApplication::getApplication() const
{
  return application_;
}

void RqtMrtaApplication::setPackage(const QString &package)
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

void RqtMrtaApplication::save(QSettings &settings) const
{
  settings.beginGroup("rqt_mrta");
  application_->save(settings);
  settings.endGroup();
}

void RqtMrtaApplication::load(QSettings &settings)
{
  settings.beginGroup("rqt_mrta");
  application_->load(settings);
  settings.endGroup();
}

void RqtMrtaApplication::reset()
{
  setPackage("");
  application_->reset();
}

void RqtMrtaApplication::write(QDataStream &stream) const
{
  application_->write(stream);
}

void RqtMrtaApplication::read(QDataStream &stream)
{
  application_->read(stream);
}

RqtMrtaApplication &RqtMrtaApplication::operator=(const RqtMrtaApplication &config)
{
  setPackage(config.package_);
  *application_ = *config.application_;
}

void RqtMrtaApplication::applicationChanged()
{
  emit changed();
}
}
}
}
