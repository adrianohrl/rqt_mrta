#include "mrta/robot.h"
#include "mrta/task.h"

namespace mrta
{
Robot::Robot(QObject* parent) : QObject(parent) {}

Robot::Robot(const Robot& robot)
    : QObject(robot.parent()), id_(robot.id_), tasks_(robot.tasks_)
{
}

Robot::~Robot() {}

QString Robot::getId() const { return id_; }

void Robot::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged();
    emit changed();
  }
}

size_t Robot::count() const { return tasks_.count(); }

Task* Robot::getTask(int index) const { return tasks_[index]; }

void Robot::addTask(Task* task)
{
  tasks_.append(task);
  emit taskAdded(task);
  emit changed();
}

void Robot::removeTask(Task* task)
{
  int index(tasks_.indexOf(task));
  tasks_.removeAt(index);
  emit taskRemoved(task);
  emit changed();
}

void Robot::clearTasks() { tasks_.clear(); }

Robot& Robot::operator=(const Robot& robot)
{
  id_ = robot.id_;
  tasks_ = robot.tasks_;
  emit changed();
}
}
