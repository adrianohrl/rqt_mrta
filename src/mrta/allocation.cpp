#include "mrta/allocation.h"
#include "mrta/robot.h"
#include "mrta/task.h"
#include <ros/time.h>

namespace mrta
{
Allocation::Allocation(QObject* parent, Task* task,
                       const QVector<Robot*>& robots)
    : QObject(parent),
      id_(task->getId() + "-" + QString::number(ros::Time::now().toSec())),
      task_(task), state_(OnHold)
{
  setRobots(robots);
  connect(task_, SIGNAL(stateChanged(int)), this, SLOT(taskStateChanged(int)));
}

Allocation::~Allocation()
{
  task_ = NULL;
  robots_.clear();
}

QString Allocation::getId() const { return id_; }

Task* Allocation::getTask() const { return task_; }

QVector<Robot*> Allocation::getRobots() const { return robots_; }

void Allocation::setRobots(const QVector<Robot*>& robots)
{
  if (robots != robots_)
  {
    if (!robots_.isEmpty())
    {
      for (size_t index(0); index < robots_.count(); index++)
      {
        disconnect(robots_[index], SIGNAL(stateChanged(int)), this,
                   SLOT(robotStateChanged(int)));
      }
    }
    robots_ = robots;
    if (!robots_.isEmpty())
    {
      for (size_t index(0); index < robots_.count(); index++)
      {
        connect(robots_[index], SIGNAL(stateChanged(int)), this,
                SLOT(robotStateChanged(int)));
      }
    }
    emit changed();
  }
}

void Allocation::setState(Allocation::State state)
{
  if (state != state_)
  {
    state_ = state;
    emit stateChanged(state);
    emit changed();
  }
}

void Allocation::robotStateChanged(int state)
{
  Robot* robot = static_cast<Robot*>(sender());
}

void Allocation::taskStateChanged(int state)
{
  Task* task = static_cast<Task*>(sender());
}
}
