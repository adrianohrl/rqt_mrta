#include <QStringList>
#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/application/robots.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Robots::Robots(QObject *parent)
  : AbstractConfig(parent)
{
}

Robots::~Robots()
{
  for (iterator it(robots_.begin()); it != robots_.end(); it++)
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
  }
}

size_t Robots::count() const
{
  return robots_.count();
}

Robot* Robots::getRobot(size_t index) const
{
  return index < robots_.count() ? robots_[index] : NULL;
}

Robot* Robots::addRobot()
{
  Robot* robot = new Robot(this);
  robots_.append(robot);
  connect(robot, SIGNAL(changed()), this,
    SLOT(robotChanged()));
  connect(robot, SIGNAL(destroyed()), this,
    SLOT(robotDestroyed()));
  emit robotAdded(robots_.count() - 1);
  emit changed();
  return robot;
}

void Robots::save(QSettings &settings) const
{
  settings.beginGroup("robots");
  for (size_t index(0); index < robots_.count(); ++index)
  {
    settings.beginGroup("robot_" + QString::number(index));
    robots_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Robots::load(QSettings &settings)
{
  settings.beginGroup("robots");
  QStringList groups(settings.childGroups());
  size_t index(0);

  for (QStringList::iterator it = groups.begin(); it != groups.end(); ++it) {
    Robot* robot = index < robots_.count() ? robot = robots_[index] : addRobot();
    settings.beginGroup(*it);
    robot->load(settings);
    settings.endGroup();
    ++index;
  }
  settings.endGroup();
  while (index < robots_.count())
  {
    removeRobot(index);
  }
}

void Robots::reset()
{
  clearRobots();
}

void Robots::write(QDataStream &stream) const
{
  for (size_t index(0); index < robots_.count(); ++index)
  {
    robots_[index]->write(stream);
  }
}

void Robots::read(QDataStream &stream)
{
  for (size_t index(0); index < robots_.count(); ++index)
  {
    robots_[index]->read(stream);
  }
}

Robots &Robots::operator=(const Robots &config)
{
  while (robots_.count() < config.robots_.count())
  {
    addRobot();
  }
  while (robots_.count() > config.robots_.count())
  {
    removeRobot(robots_.count() - 1);
  }
  for (size_t index(0); index < robots_.count(); ++index)
  {
    *robots_[index] = *config.robots_[index];
  }
}

void Robots::robotChanged()
{
  for (size_t index(0); index < robots_.count(); ++index)
  {
    if (robots_[index] == sender())
    {
      emit robotChanged(index);
      break;
    }
  }
  emit changed();
}

void Robots::robotDestroyed()
{
  int index(robots_.indexOf(static_cast<Robot*>(sender())));
  if (index >= 0)
  {
    robots_.remove(index);
    emit robotRemoved(index);
    emit changed();
  }
}
}
}
}
