#include "rqt_mrta/config/application/application.h"
#include "rqt_mrta/config/application/robots.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Application::Application(QObject *parent)
  : AbstractConfig(parent), robots_(new Robots(this))
{
  connect(robots_, SIGNAL(changed()), this, SLOT(robotsChanged()));
}

Application::~Application()
{
  if (robots_)
  {
    delete robots_;
    robots_ = NULL;
  }
}

QString Application::getName() const
{
  return name_;
}

QString Application::getUrl() const
{
  return url_;
}

Robots *Application::getRobots() const
{
  return robots_;
}

void Application::setName(const QString &name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
  }
}

void Application::setUrl(const QString &url)
{
  if (url != url_)
  {
    url_ = url;
    emit urlChanged(url);
    emit changed();
  }
}

void Application::save(QSettings &settings) const
{
  settings.beginGroup("application");
  settings.setValue("name", name_);
  settings.setValue("url", url_);
  settings.endGroup();
}

void Application::load(QSettings &settings)
{
  settings.beginGroup("application");
  setName(settings.value("name").toString());
  setUrl(settings.value("url").toString());
  settings.endGroup();
}

void Application::reset()
{
  setName("");
  setUrl("");
}

void Application::write(QDataStream &stream) const
{
  stream << name_;
  stream << url_;
}

void Application::read(QDataStream &stream)
{
  QString name, url;
  stream >> name;
  setName(name);
  stream >> url;
  setUrl(url);
}

Application &Application::operator=(const Application &config)
{
  setName(config.name_);
  setUrl(config.url_);
}

void Application::robotsChanged()
{
  emit changed();
}
}
}
}
