#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/application/tasks.h"
#include "rqt_mrta/config/config.h"
#include "rqt_mrta/config/param.h"
#include "rqt_mrta/config/param_interface.h"
#include "rqt_mrta/config/params.h"
#include "rqt_mrta/config/params_array.h"
#include "utilities/exception.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Robot::Robot(QObject* parent)
    : AbstractConfig(parent), tasks_(new Tasks(this)), config_(new Config(this))
{
  connect(tasks_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(tasks_, SIGNAL(taskIdChanged(size_t, const QString&)), this,
          SLOT(taskChanged(size_t, const QString&)));
  connect(tasks_, SIGNAL(added(size_t)), this, SLOT(taskAdded(size_t)));
  connect(tasks_, SIGNAL(removed(const QString&)), this,
          SLOT(taskRemoved(const QString&)));
  connect(tasks_, SIGNAL(cleared()), this, SLOT(tasksCleared()));
  connect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Robot::~Robot()
{
  ROS_INFO_STREAM("[~Robot] before ...");
  if (tasks_)
  {
    delete tasks_;
    tasks_ = NULL;
  }
  clearArrays();
  if (config_)
  {
    delete config_;
    config_ = NULL;
  }
  ROS_INFO_STREAM("[~Robot] after ...");
}

QString Robot::getId() const { return id_; }

Tasks* Robot::getTasks() const { return tasks_; }

Config* Robot::getConfig() const { return config_; }

void Robot::setConfig(Config* config)
{
  if (!config)
  {
    return;
  }
  ROS_INFO("[Robot::setConfig] to aki");
  clearArrays();
  *config_ = *config;
  for (size_t index(0); index < config_->count(); index++)
  {
    if (config_->getChild(index)->isArray())
    {
      throw utilities::Exception("ParamArrays must have a Params parent.");
    }
    else if (config_->getChild(index)->isParams())
    {
      findArrays(static_cast<Params*>(config_->getChild(index)));
    }
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

void Robot::save(QSettings& settings) const
{
  settings.setValue("id", id_);
  tasks_->save(settings);
  config_->save(settings);
}

void Robot::load(QSettings& settings)
{
  setId(settings.value("id").toString());
  tasks_->load(settings);
  config_->load(settings);
}

void Robot::reset()
{
  setId("");
  tasks_->reset();
  config_->reset();
}

void Robot::write(QDataStream& stream) const
{
  stream << id_;
  tasks_->write(stream);
  config_->write(stream);
}

void Robot::read(QDataStream& stream)
{
  QString id;
  stream >> id;
  setId(id);
  tasks_->read(stream);
  config_->read(stream);
}

Robot& Robot::operator=(const Robot& config)
{
  setId(config.id_);
  *tasks_ = *config.tasks_;
  *config_ = *config.config_;
  return *this;
}

QString Robot::validate() const
{
  if (id_.isEmpty())
  {
    return "The robot id must not be empty.";
  }
  if (id_.contains(' '))
  {
    return "The robot id must not contain <space>.";
  }
  return tasks_->validate();
}

void Robot::findArrays(Params* parent)
{
  ROS_WARN_STREAM("[Robot::findArrays] parent: " << parent->getFullName().toStdString());
  for (size_t index(0); index < parent->count(); index++)
  {
    ROS_WARN_STREAM("[Robot::findArrays] child: " << parent->getChild(index)->getFullName().toStdString());
    if (parent->getChild(index)->isArray())
    {
      ParamInterface* size = parent->getParam("size");
      if (!size || !size->isParam())
      {
        throw utilities::Exception("The ParamsArray's parent must have a Param named size.");
      }
      ParamsArray* array = static_cast<ParamsArray*>(parent->getChild(index));
      ROS_WARN_STREAM("[Robot::findArrays] found array not removed yet: " << parent->params_.count());
      //parent->params_.remove(parent->params_.indexOf(array));
      parent->removeParam(array->getName());
      ROS_WARN_STREAM("[Robot::findArrays] found array removed: " << parent->params_.count());
      ROS_WARN_STREAM("[Robot::findArrays] found array: " << parent->getFullName().toStdString());
      connect(size, SIGNAL(valueChanged(const QString&, const QVariant&)), this,
              SLOT(arraySizeChanged(const QString&, const QVariant&)));
      arrays_.insert(static_cast<Param*>(size), array);
      //findArrays(array);
    }
    else if (parent->getChild(index)->isParams())
    {
      findArrays(static_cast<Params*>(parent->getChild(index)));
    }
  }
}

void Robot::clearArrays()
{
  for (iterator it(arrays_.begin()); it != arrays_.end(); it++)
  {
    arrays_[it.key()]  = NULL;
    arrays_.remove(it.key());
  }
  arrays_.clear();
}

void Robot::taskChanged(size_t task_index, const QString& task_id)
{
  emit taskIdChanged(task_index, task_id);
}

void Robot::taskAdded(size_t task_index) { emit added(task_index); }

void Robot::taskRemoved(const QString& task_id) { emit removed(task_id); }

void Robot::tasksCleared() { emit cleared(); }

void Robot::arraySizeChanged(const QString& full_name, const QVariant& value)
{
  ROS_ERROR_STREAM("[Robot::arraySizeChanged] name: "
                   << full_name.toStdString()
                   << ", value: " << value.toString().toStdString());
  Param* size = static_cast<Param*>(sender());
  ROS_ERROR_STREAM("[Robot::arraySizeChanged] size.");
  ParamsArray* array = arrays_[size];
  array->createParams(value.toInt());
  findArrays(array->getParentParam());
  ROS_ERROR_STREAM("[Robot::arraySizeChanged] ended.");
}
}
}
}
