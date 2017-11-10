#include <QStringList>
#include "rqt_mrta/config/architecture/config.h"
#include "rqt_mrta/config/architecture/param_factory.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Config::Config(QObject* parent) : AbstractConfig(parent) {}

Config::~Config()
{
  for (size_t i(0); i < params_.count(); i++)
  {
    if (params_[i])
    {
      delete params_[i];
      params_[i] = NULL;
    }
  }
  params_.clear();
}

QString Config::getId() const { return id_; }

void Config::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

ParamInterface* Config::getParam(const QString& full_name) const
{
  QStringList names(full_name.split("/"));
  QString root_name(names[0]);
  if (root_name.isEmpty())
  {
    return NULL;
  }
  names.removeFirst();
  for (size_t i(0); i < params_.count(); i++)
  {
    if (params_[i]->getName() == root_name)
    {
      return !names.isEmpty() ? params_[i]->getParam(names.join("/"))
                              : params_[i];
    }
  }
  return NULL;
}

void Config::addParam(ParamInterface* param)
{
  params_.append(param);
  /*connect(param, SIGNAL(nameChanged(const QString&, const QString&)), this,
          SLOT(paramNameChanged(const QString&, const QString&)));
  connect(param, SIGNAL(typeChanged(const QString&, const QMetaType::Type&)),
          this, SLOT(paramTypeChanged(const QString&, const QMetaType::Type&)));
  connect(param, SIGNAL(valueChanged(const QString&, const QVariant&)), this,
          SLOT(paramValueChanged(const QString&, const QVariant&)));
  connect(
      param, SIGNAL(defaultValueChanged(const QString&, const QVariant&)),
      this, SLOT(paramDefaultValueChanged(const QString&, const QVariant&)));
  connect(param, SIGNAL(toolTipChanged(const QString&, const QString&)), this,
          SLOT(paramToolTipChanged(const QString&, const QString&)));
  connect(param, SIGNAL(added(const QString&)), this,
          SLOT(paramAdded(const QString&)));
  connect(param, SIGNAL(removed(const QString&)), this,
          SLOT(paramRemoved(const QString&)));
  connect(param, SIGNAL(cleared(const QString&)), this,
          SLOT(paramCleared(const QString&)));
  connect(param, SIGNAL(destroyed()), this, SLOT(paramDestroyed()));*/
  emit added(param->getFullName());
  emit changed();
}

void Config::removeParam(const QString& full_name)
{
  ParamInterface* param = getParam(full_name);
  if (param)
  {
    ParamInterface* parent = param->getParentParam();
    if (parent)
    {
      parent->removeParam(param->getName());
    }
    else
    {
      size_t index(params_.indexOf(param));
      if (index == -1)
      {
        return;
      }
      ROS_INFO_STREAM("[Config] removing param "
                      << (params_[index] ? params_[index]->getFullName() : "-")
                             .toStdString() << " at " << index);
      if (params_[index])
      {
        delete params_[index];
        params_[index] = NULL;
      }
      params_.remove(index);
    }
    emit removed(full_name);
    emit changed();
  }
}

void Config::clearParams()
{
  if (!params_.isEmpty())
  {
    for (size_t i(0); i < params_.count(); i++)
    {
      if (params_[i])
      {
        delete params_[i];
        params_[i] = NULL;
      }
    }
    params_.clear();
    emit cleared("");
    emit changed();
  }
}

void Config::clearParams(const QString &full_name)
{
  if (!full_name.isEmpty())
  {
    ParamInterface* param = getParam(full_name);
    param->clearParams();
  }
  else if (!params_.isEmpty())
  {
    clearParams();
  }
}

bool Config::contains(const QString& full_name) const
{
  QStringList names(full_name.split("/"));
  for (size_t index(0); index < params_.count(); index++)
  {
    if (params_[index]->getName() == names[0])
    {
      names.removeFirst();
      return names.isEmpty() || params_[index]->contains(names.join("/"));
    }
  }
  return false;
}

size_t Config::count() const
{
  return params_.count();
}

size_t Config::count(const QString &full_name) const
{
  if (full_name.isEmpty())
  {
    return params_.count();
  }
  ParamInterface* param = getParam(full_name);
  return param->count();
}

bool Config::isEmpty() const { return params_.isEmpty(); }

bool Config::isEmpty(const QString &full_name) const
{
  if (full_name.isEmpty())
  {
    return params_.isEmpty();
  }
  ParamInterface* param = getParam(full_name);
  return param->isEmpty();
}

void Config::save(QSettings& settings) const
{
  settings.setValue("id", id_);
  for (size_t index(0); index < params_.count(); ++index)
  {
    settings.beginGroup(params_[index]->getGroupName() + "_" + QString::number(index));
    params_[index]->save(settings);
    settings.endGroup();
  }
}

void Config::load(QSettings& settings)
{
  setId(settings.value("id").toString());
  QStringList groups(Params::sortGroups(settings.childGroups()));
  clearParams();
  for (size_t index(0); index < groups.count(); index++)
  {
    ParamInterface* param = ParamFactory::newInstance(groups[index]);
    addParam(param);
    settings.beginGroup(param->getGroupName() + "_" + QString::number(index));
    param->load(settings);
    settings.endGroup();
  }
}

void Config::reset()
{
  for (size_t index(0); index < params_.count(); index++)
  {
    params_[index]->reset();
  }
}

void Config::write(QDataStream& stream) const
{
  stream << id_;
  stream << params_.count();
  for (size_t index(0); index < params_.count(); ++index)
  {
    params_[index]->write(stream);
  }
}

void Config::read(QDataStream& stream)
{
  quint64 count;
  QString id;
  stream >> id;
  setId(id);
  stream >> count;
  clearParams();
  for (size_t index(0); index < count; ++index)
  {
    params_[index]->read(stream);
  }
}

Config& Config::operator=(const Config& config)
{
  setId(config.id_);
  clearParams();
  for (size_t index(0); index < config.params_.count(); index++)
  {
    addParam(config.params_[index]->clone());
  }
  return *this;
}

QString Config::validate() const
{
  if (id_.isEmpty())
  {
    return "The config name must not be empty.";
  }
  if (id_.contains(' '))
  {
    return "The config name must not contain <space>.";
  }
  if (params_.isEmpty())
  {
    return "Enter the config parameters.";
  }
  QString validation;
  for (size_t i(0); i < params_.count(); i++)
  {
    validation = params_[i]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

void Config::paramAdded(const QString& full_name)
{
  emit added(full_name);
  emit changed();
}

void Config::paramRemoved(const QString& full_name)
{
  emit removed(full_name);
  emit changed();
}

void Config::paramCleared(const QString& full_name)
{
  emit cleared(full_name);
  emit changed();
}

void Config::paramNameChanged(const QString& previous_full_name,
                              const QString& name)
{
  emit paramNameChanged(previous_full_name, name);
  emit changed();
}

void Config::paramTypeChanged(const QString& full_name,
                              const QMetaType::Type& type)
{
  emit paramTypeChanged(full_name, type);
  emit changed();
}

void Config::paramValueChanged(const QString& full_name, const QVariant& value)
{
  emit paramValueChanged(full_name, value);
  emit changed();
}

void Config::paramDefaultValueChanged(const QString& full_name,
                                      const QVariant& default_value)
{
  emit paramDefaultValueChanged(full_name, default_value);
  emit changed();
}

void Config::paramToolTipChanged(const QString& full_name,
                                 const QString& tool_tip)
{
  emit paramToolTipChanged(full_name, tool_tip);
  emit changed();
}

void Config::paramDestroyed()
{
  ParamInterface* param = static_cast<ParamInterface*>(sender());
  int index(params_.indexOf(param));
  if (index != -1)
  {
    QString full_name(param->getFullName());
    params_.remove(index);
    emit removed(full_name);
    emit changed();
  }
}
}
}
}
