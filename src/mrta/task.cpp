#include "mrta/task.h"

namespace mrta
{
Task::Task(QObject* parent) : QObject(parent) {}

Task::Task(const Task& task) : QObject(task.parent()), id_(task.id_) {}

Task::~Task() {}

QString Task::getId() const { return id_; }

void Task::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged();
    emit changed();
  }
}

Task& Task::operator=(const Task& task)
{
  id_ = task.id_;
  emit changed();
}
}
