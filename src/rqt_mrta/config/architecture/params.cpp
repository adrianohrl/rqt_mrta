#include <QStringList>
#include <ros/console.h>
#include "rqt_mrta/config/architecture/params.h"
#include "rqt_mrta/config/architecture/param_factory.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Params::Params(Params *parent) : ParamInterface("params", parent) {}

Params::Params(const QString& group_name, Params* parent)
    : ParamInterface(group_name, parent)
{
}

Params::~Params()
{
  ROS_INFO_STREAM("[~Params] before ...");
  for (size_t index(0); index < params_.count(); index++)
  {
    if (params_[index])
    {
      delete params_[index];
      params_[index] = NULL;
    }
  }
  params_.clear();
  ROS_INFO_STREAM("[~Params] after ...");
}

ParamInterface* Params::getParam(const QString& full_name) const
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

void Params::addParam(ParamInterface* param)
{
  params_.append(param);
  connect(param, SIGNAL(changed()), this, SLOT(paramChanged()));
  connect(param, SIGNAL(nameChanged(const QString&, const QString&)), this,
          SIGNAL(nameChanged(const QString&, const QString&)));
  connect(param, SIGNAL(typeChanged(const QString&, const QMetaType::Type&)),
          this, SIGNAL(typeChanged(const QString&, const QMetaType::Type&)));
  connect(param, SIGNAL(valueChanged(const QString&, const QVariant&)), this,
          SIGNAL(valueChanged(const QString&, const QVariant&)));
  connect(
      param, SIGNAL(defaultValueChanged(const QString&, const QVariant&)),
      this, SIGNAL(defaultValueChanged(const QString&, const QVariant&)));
  connect(param, SIGNAL(toolTipChanged(const QString&, const QString&)), this,
          SIGNAL(toolTipChanged(const QString&, const QString&)));
  connect(param, SIGNAL(added(const QString&)), this,
          SIGNAL(added(const QString&)));
  connect(param, SIGNAL(removed(const QString&)), this,
          SIGNAL(removed(const QString&)));
  connect(param, SIGNAL(cleared(const QString&)), this,
          SIGNAL(cleared(const QString&)));
  connect(param, SIGNAL(destroyed()), this, SLOT(paramDestroyed()));
  emit added(param->getFullName());
  emit changed();
}

void Params::removeParam(const QString& full_name)
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
      ROS_INFO_STREAM("[Params] removing param "
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

void Params::clearParams()
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
    emit cleared(getFullName());
    emit changed();
  }
}

bool Params::contains(const QString& full_name) const
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

size_t Params::count() const
{
  return params_.count();
}

bool Params::isEmpty() const { return params_.isEmpty(); }

void Params::save(QSettings& settings) const
{
  ParamInterface::save(settings);
  for (size_t index(0); index < params_.count(); ++index)
  {
    settings.beginGroup(params_[index]->getGroupName() + "_" + QString::number(index));
    params_[index]->save(settings);
    settings.endGroup();
  }
}

void Params::load(QSettings& settings)
{
  ParamInterface::load(settings);
  QStringList groups(Params::sortGroups(settings.childGroups()));
  clearParams();
  for (size_t index(0); index < groups.count(); index++)
  {
    ParamInterface* param = ParamFactory::newInstance(groups[index], this);
    addParam(param);
    settings.beginGroup(param->getGroupName() + "_" + QString::number(index));
    param->load(settings);
    settings.endGroup();
  }
}

void Params::reset()
{
  for (size_t index(0); index < params_.count(); index++)
  {
    params_[index]->reset();
  }
}

void Params::write(QDataStream& stream) const
{
  ParamInterface::write(stream);
  stream << params_.count();
  for (size_t index(0); index < params_.count(); ++index)
  {
    params_[index]->write(stream);
  }
}

void Params::read(QDataStream& stream)
{
  quint64 count;
  ParamInterface::read(stream);
  stream >> count;
  clearParams();
  for (size_t index(0); index < count; ++index)
  {
    params_[index]->read(stream);
  }
}

Params& Params::operator=(const Params& config)
{
  ParamInterface::operator=(config);
  clearParams();
  for (size_t index(0); index < config.params_.count(); index++)
  {
    addParam(config.params_[index]->clone());
  }
  return *this;
}

ParamInterface *Params::clone() const
{
  Params* params = new Params();
  *params = *this;
  return params;
}

QString Params::validate() const
{
  if (params_.isEmpty())
  {
    return "Enter the params parameters.";
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
}/*

void Params::paramAdded(const QString& full_name)
{
  emit added(full_name);
  emit changed();
}

void Params::paramRemoved(const QString& full_name)
{
  emit removed(full_name);
  emit changed();
}

void Params::paramCleared(const QString& full_name)
{
  emit cleared(full_name);
  emit changed();
}

void Params::paramNameChanged(const QString& previous_full_name,
                              const QString& name)
{
  emit paramNameChanged(previous_full_name, name);
  emit changed();
}

void Params::paramTypeChanged(const QString& full_name,
                              const QMetaType::Type& type)
{
  emit paramTypeChanged(full_name, type);
  emit changed();
}

void Params::paramValueChanged(const QString& full_name, const QVariant& value)
{
  emit paramValueChanged(full_name, value);
  emit changed();
}

void Params::paramDefaultValueChanged(const QString& full_name,
                                      const QVariant& default_value)
{
  emit paramDefaultValueChanged(full_name, default_value);
  emit changed();
}

void Params::paramToolTipChanged(const QString& full_name,
                                 const QString& tool_tip)
{
  emit paramToolTipChanged(full_name, tool_tip);
  emit changed();
}*/

void Params::paramDestroyed()
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

QStringList Params::sortGroups(const QStringList &groups)
{
  QStringList sorted_groups;
  for (size_t index(0); index < groups.count(); index++)
  {
    QString group(groups.filter("_" + QString::number(index)).first());
    if (group.isEmpty())
    {
      ROS_ERROR("Invalid param index.");
      return sorted_groups;
    }
    sorted_groups.append(group);
  }
  return sorted_groups;
}
}
}
}
