#include "mrta/robot.h"
#include "mrta/task.h"
#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/architecture/busy_robots.h"
#include "rqt_mrta/config/architecture/idle_robots.h"
#include "rqt_mrta/config/architecture/robots.h"

namespace mrta
{
Robot::Robot(QObject* parent)
    : QObject(parent), config_(NULL), busy_config_(NULL), idle_config_(NULL),
      state_(Offline)
{
}

Robot::Robot(QObject* parent, Config* config, TopicsConfig* topics_config)
    : QObject(parent), id_(config->getId()), config_(NULL), busy_config_(NULL),
      idle_config_(NULL), state_(Offline)
{
  setConfig(config);
  setBusyTopicConfig(topics_config->getBusyRobots());
  setIdleTopicConfig(topics_config->getIdleRobots());
}

Robot::Robot(const Robot& robot)
    : QObject(robot.parent()), config_(NULL), busy_config_(NULL),
      idle_config_(NULL), id_(robot.id_), tasks_(robot.tasks_),
      state_(robot.state_)
{
  setConfig(robot.config_);
  setBusyTopicConfig(robot.busy_config_);
  setIdleTopicConfig(robot.idle_config_);
}

Robot::~Robot()
{
  config_ = NULL;
  busy_config_ = NULL;
  idle_config_ = NULL;
}

Robot::Config* Robot::getConfig() const { return config_; }

Robot::BusyTopicConfig* Robot::getBusyTopicConfig() const
{
  return busy_config_;
}

Robot::IdleTopicConfig* Robot::getIdleTopicConfig() const
{
  return idle_config_;
}

void Robot::setConfig(Robot::Config* config)
{
  if (config != config_)
  {
    if (config_)
    {
      disconnect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
      disconnect(config_, SIGNAL(destroyed()), this, SLOT(configDestroyed()));
      disconnect(config_, SIGNAL(idChanged(const QString&)), this,
                 SLOT(setId(const QString&)));
    }
    config_ = config;
    if (config_)
    {
      connect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
      connect(config_, SIGNAL(destroyed()), this, SLOT(configDestroyed()));
      connect(config_, SIGNAL(idChanged(const QString&)), this,
              SLOT(setId(const QString&)));
      setId(config_->getId());
      clearTasks();
      for (size_t index(0); index < config->getTasks()->count(); index++)
      {
        addTask(new Task(this, config->getTasks()->getTask(index)));
      }
    }
  }
}

void Robot::setBusyTopicConfig(Robot::BusyTopicConfig* config)
{
  if (busy_config_ != config)
  {
    if (busy_config_)
    {
    }
    busy_config_ = config;
    if (busy_config_)
    {
      connect(busy_config_, SIGNAL(changed()), this, SIGNAL(changed()));
      connect(busy_config_, SIGNAL(destroyed()), this, SLOT(busyTopicConfigDestroyed()));
    }
    emit changed();
  }
}

void Robot::setIdleTopicConfig(Robot::IdleTopicConfig* config)
{
  if (idle_config_ != config)
  {
    if (idle_config_)
    {
    }
    idle_config_ = config;
    if (idle_config_)
    {
      connect(idle_config_, SIGNAL(changed()), this, SIGNAL(changed()));
      connect(idle_config_, SIGNAL(destroyed()), this, SLOT(idleTopicConfigDestroyed()));
    }
    emit changed();
  }
}

QString Robot::getId() const { return id_; }

Robot::State Robot::getState() const { return state_; }

void Robot::setState(Robot::State state)
{
  if (state != state_)
  {
    state_ = state;
    switch (state_)
    {
    case Idle:
      emit idle();
      break;
    case Busy:
      emit busy();
      break;
    case Offline:
      emit offline();
      break;
    }
    emit stateChanged(state);
    emit changed();
  }
}

void Robot::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

size_t Robot::count() const { return tasks_.count(); }

Task* Robot::getTask(int index) const { return tasks_[index]; }

void Robot::addTask(Task* task)
{
  tasks_.append(task);
  connect(task, SIGNAL(changed()), this, SLOT(changed()));
  connect(task, SIGNAL(changed()), this, SLOT(taskChanged()));
  connect(task, SIGNAL(idChanged(const QString&)), this,
          SLOT(taskIdChanged(const QString&)));
  connect(task, SIGNAL(destroyed()), this, SLOT(taskDestroyed()));
  emit added(tasks_.count() - 1);
  emit changed();
}

void Robot::removeTask(Task* task)
{
  size_t index(tasks_.indexOf(task));
  QString task_id(task->getId());
  tasks_.remove(index);
  emit removed(task_id);
  emit changed();
}

void Robot::clearTasks() { tasks_.clear(); }

Robot& Robot::operator=(const Robot& robot)
{
  id_ = robot.id_;
  tasks_ = robot.tasks_;
  emit changed();
}

void Robot::configDestroyed() { config_ = NULL; }

void Robot::busyTopicConfigDestroyed() { busy_config_ = NULL; }

void Robot::idleTopicConfigDestroyed() { idle_config_ = NULL; }

void Robot::taskDestroyed()
{
  Task* task = static_cast<Task*>(sender());
  int index(tasks_.indexOf(task));
  if (index != -1)
  {
    QString task_id(task->getId());
    tasks_.remove(index);
    emit removed(task_id);
    emit changed();
  }
}
}
