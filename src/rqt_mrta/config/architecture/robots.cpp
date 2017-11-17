#include "rqt_mrta/config/architecture/robots.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Robots::Robots(QObject* parent)
    : AbstractConfig(parent), busy_robots_(new BusyRobots(this)),
      idle_robots_(new IdleRobots(this)), launch_(new RobotLaunch(this))
{
  connect(busy_robots_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(idle_robots_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(launch_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Robots::~Robots()
{
  ROS_INFO_STREAM("[~Robots] before ...");
  if (busy_robots_)
  {
    delete busy_robots_;
    busy_robots_ = NULL;
  }
  if (idle_robots_)
  {
    delete idle_robots_;
    idle_robots_ = NULL;
  }
  if (launch_)
  {
    delete launch_;
    launch_ = NULL;
  }
  ROS_INFO_STREAM("[~Robots] after ...");
}

QString Robots::getType() const
{
  return type_;
}

QString Robots::getConfigId() const
{
  return config_id_;
}

BusyRobots* Robots::getBusyRobots() const { return busy_robots_; }

IdleRobots* Robots::getIdleRobots() const { return idle_robots_; }

RobotLaunch* Robots::getLaunch() const { return launch_; }

void Robots::setType(const QString &type)
{
  if (type != type_)
  {
    type_ = type;
    emit typeChanged(type);
    emit changed();
  }
}

void Robots::setConfigId(const QString &config_id)
{
  if (config_id != config_id_)
  {
    config_id_ = config_id;
    emit configIdChanged(config_id);
    emit changed();
  }
}

void Robots::save(QSettings& settings) const
{
  settings.beginGroup("robots");
  settings.setValue("type", type_);
  settings.setValue("config_id", config_id_);
  busy_robots_->save(settings);
  idle_robots_->save(settings);
  launch_->save(settings);
  settings.endGroup();
}

void Robots::load(QSettings& settings)
{
  settings.beginGroup("robots");
  setType(settings.value("type").toString());
  setConfigId(settings.value("config_id").toString());
  busy_robots_->load(settings);
  idle_robots_->load(settings);
  launch_->load(settings);
  settings.endGroup();
}

void Robots::reset()
{
  setType("");
  setConfigId("");
  busy_robots_->reset();
  idle_robots_->reset();
  launch_->reset();
}

void Robots::write(QDataStream& stream) const
{
  stream << type_;
  stream << config_id_;
  busy_robots_->write(stream);
  idle_robots_->write(stream);
  launch_->write(stream);
}

void Robots::read(QDataStream& stream)
{
  QString type;
  stream >> type;
  setType(type);
  QString config_id;
  stream >> config_id;
  setConfigId(config_id);
  busy_robots_->read(stream);
  idle_robots_->read(stream);
  launch_->read(stream);
}

Robots& Robots::operator=(const Robots& config)
{
  setType(config.type_);
  setConfigId(config.config_id_);
  *busy_robots_ = *config.busy_robots_;
  *idle_robots_ = *config.idle_robots_;
  *launch_ = *config.launch_;
  return *this;
}
}
}
}
