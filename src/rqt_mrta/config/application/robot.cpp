#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/application/tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Robot::Robot(QObject *parent)
  : AbstractConfig(parent), tasks_(new Tasks(this))
{
  connect(tasks_, SIGNAL(changed()), this, SLOT(tasksChanged()));
}

Robot::~Robot()
{
  if (tasks_)
  {
    delete tasks_;
    tasks_ = NULL;
  }
}

QString Robot::getId() const
{
  return id_;
}

Tasks *Robot::getTasks() const
{
  return tasks_;
}

void Robot::setId(const QString &id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

void Robot::save(QSettings &settings) const
{
  settings.setValue("id", id_);
  tasks_->save(settings);
}

void Robot::load(QSettings &settings)
{
  setId(settings.value("id").toString());
  tasks_->load(settings);
}

void Robot::reset()
{
  setId("");
  tasks_->reset();
}

void Robot::write(QDataStream &stream) const
{
  stream << id_;
  tasks_->write(stream);
}

void Robot::read(QDataStream &stream)
{
  QString id;
  stream >> id;
  setId(id);
}

Robot &Robot::operator=(const Robot &config)
{
  setId(config.id_);
  *tasks_ = *config.tasks_;
}

void Robot::tasksChanged()
{
  emit changed();
}
}
}
}
