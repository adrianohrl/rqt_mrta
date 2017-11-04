#include <QStringList>
#include "rqt_mrta/config/application/task.h"
#include "rqt_mrta/config/application/tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Tasks::Tasks(QObject *parent)
  : AbstractConfig(parent)
{
}

Tasks::~Tasks()
{
  for (iterator it(tasks_.begin()); it != tasks_.end(); it++)
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
  }
}

size_t Tasks::count() const
{
  return tasks_.count();
}

Task* Tasks::getTask(size_t index) const
{
  return index < tasks_.count() ? tasks_[index] : NULL;
}

Task* Tasks::addTask()
{
  Task* task = new Task(this);
  tasks_.append(task);
  connect(task, SIGNAL(changed()), this,
    SLOT(taskChanged()));
  connect(task, SIGNAL(destroyed()), this,
    SLOT(taskDestroyed()));
  emit taskAdded(tasks_.count() - 1);
  emit changed();
  return task;
}

void Tasks::save(QSettings &settings) const
{
  settings.beginGroup("tasks");
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    settings.beginGroup("task_" + QString::number(index));
    tasks_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Tasks::load(QSettings &settings)
{
  settings.beginGroup("tasks");
  QStringList groups(settings.childGroups());
  size_t index(0);

  for (QStringList::iterator it = groups.begin(); it != groups.end(); ++it) {
    Task* task = index < tasks_.count() ? task = tasks_[index] : addTask();
    settings.beginGroup(*it);
    task->load(settings);
    settings.endGroup();
    ++index;
  }
  settings.endGroup();
  while (index < tasks_.count())
  {
    removeTask(index);
  }
}

void Tasks::reset()
{
  clearTasks();
}

void Tasks::write(QDataStream &stream) const
{
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    tasks_[index]->write(stream);
  }
}

void Tasks::read(QDataStream &stream)
{
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    tasks_[index]->read(stream);
  }
}

Tasks &Tasks::operator=(const Tasks &config)
{
  while (tasks_.count() < config.tasks_.count())
  {
    addTask();
  }
  while (tasks_.count() > config.tasks_.count())
  {
    removeTask(tasks_.count() - 1);
  }
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    *tasks_[index] = *config.tasks_[index];
  }
}

void Tasks::taskChanged()
{
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    if (tasks_[index] == sender())
    {
      emit taskChanged(index);
      break;
    }
  }
  emit changed();
}

void Tasks::taskDestroyed()
{
  int index(tasks_.indexOf(static_cast<Task*>(sender())));
  if (index >= 0)
  {
    tasks_.remove(index);
    emit taskRemoved(index);
    emit changed();
  }
}
}
}
}
