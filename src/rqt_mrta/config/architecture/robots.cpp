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
  connect(busy_robots_, SIGNAL(changed()), this, SLOT(busyRobotsChanged()));
  connect(idle_robots_, SIGNAL(changed()), this, SLOT(idleRobotsChanged()));
  connect(launch_, SIGNAL(changed()), this, SLOT(launchChanged()));
}

Robots::~Robots()
{
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
}

BusyRobots* Robots::getBusyRobots() const { return busy_robots_; }

IdleRobots* Robots::getIdleRobots() const { return idle_robots_; }

RobotLaunch* Robots::getLaunch() const { return launch_; }

void Robots::save(QSettings& settings) const
{
  settings.beginGroup("robots");
  busy_robots_->save(settings);
  idle_robots_->save(settings);
  launch_->save(settings);
  settings.endGroup();
}

void Robots::load(QSettings& settings)
{
  settings.beginGroup("robots");
  busy_robots_->load(settings);
  idle_robots_->load(settings);
  launch_->load(settings);
  settings.endGroup();
}

void Robots::reset()
{
  busy_robots_->reset();
  idle_robots_->reset();
  launch_->reset();
}

void Robots::write(QDataStream& stream) const
{
  busy_robots_->write(stream);
  idle_robots_->write(stream);
  launch_->write(stream);
}

void Robots::read(QDataStream& stream)
{
  busy_robots_->read(stream);
  idle_robots_->read(stream);
  launch_->read(stream);
}

Robots& Robots::operator=(const Robots& config)
{
  *busy_robots_ = *config.busy_robots_;
  *idle_robots_ = *config.idle_robots_;
  *launch_ = *config.launch_;
}

void Robots::busyRobotsChanged() { emit changed(); }

void Robots::idleRobotsChanged() { emit changed(); }

void Robots::launchChanged() { emit changed(); }
}
}
}
