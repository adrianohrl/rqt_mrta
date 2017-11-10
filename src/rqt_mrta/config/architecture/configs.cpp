#include <QStringList>
#include <ros/console.h>
#include "rqt_mrta/config/architecture/configs.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Configs::Configs(QObject* parent) : AbstractConfig(parent) {}

Configs::~Configs() {}

Config* Configs::getConfig(size_t index) const
{
  return index < configs_.count() ? configs_[index] : NULL;
}

Config* Configs::addConfig()
{
  Config* config = new Config(this);
  configs_.append(config);
  connect(config, SIGNAL(changed()), this, SLOT(configChanged()));
  connect(config, SIGNAL(idChanged(const QString&)), this,
          SLOT(configIdChanged(const QString&)));
  connect(config, SIGNAL(added(const QString&)), this,
          SLOT(configAdded(const QString&)));
  connect(config, SIGNAL(removed(const QString&)), this,
          SLOT(configRemoved(const QString&)));
  connect(config, SIGNAL(cleared(const QString&)), this,
          SLOT(configCleared(const QString&)));
  connect(config, SIGNAL(nameChanged(const QString&, const QString&)), this,
          SLOT(configNameChanged(const QString&, const QString&)));
  connect(config, SIGNAL(typeChanged(const QString&, const QMetaType::Type&)),
          this,
          SLOT(configTypeChanged(const QString&, const QMetaType::Type&)));
  connect(config, SIGNAL(valueChanged(const QString&, const QVariant&)), this,
          SLOT(configValueChanged(const QString&, const QVariant&)));
  connect(config, SIGNAL(defaultValueChanged(const QString&, const QVariant&)),
          this,
          SLOT(configDefaultValueChanged(const QString&, const QVariant&)));
  connect(config, SIGNAL(toolTipChanged(const QString&, const QString&)), this,
          SLOT(configToolTipChanged(const QString&, const QString&)));
  connect(config, SIGNAL(destroyed()), this, SLOT(configDestroyed()));
  emit added(configs_.count() - 1);
  emit changed();
  return config;
}

void Configs::removeConfig(Config* config)
{
  removeConfig(configs_.indexOf(config));
}

void Configs::removeConfig(size_t index)
{
  if (index >= 0 && index < configs_.count())
  {
    QString config_id(configs_[index]->getId());
    configs_.remove(index);
    emit removed(config_id);
    emit changed();
  }
}

void Configs::clearConfigs()
{
  if (!configs_.isEmpty())
  {
    for (size_t i(0); i < configs_.count(); ++i)
    {
      if (configs_[i])
      {
        delete configs_[i];
        configs_[i] = NULL;
      }
    }
    configs_.clear();
    emit cleared();
    emit changed();
  }
}

bool Configs::contains(const QString& id) const
{
  for (size_t i(0); i < configs_.count(); i++)
  {
    if (configs_[i]->getId() == id)
    {
      return true;
    }
  }
  return false;
}

size_t Configs::count() const { return configs_.count(); }

bool Configs::isEmpty() const { return configs_.isEmpty(); }

void Configs::save(QSettings& settings) const
{
  settings.beginGroup("configs");
  for (size_t index(0); index < configs_.count(); ++index)
  {
    configs_[index]->save(settings);
  }
  settings.endGroup();
}

void Configs::load(QSettings& settings)
{
  settings.beginGroup("configs");
  clearConfigs();
  QStringList groups(settings.childGroups());
  for (int i(0); i < groups.count(); i++)
  {
    ROS_INFO_STREAM("[Configs::load] group " << i << ": "
                                             << groups[i].toStdString());
  }

  for (size_t i(0); i < groups.count(); i++)
  {
    Config* config = addConfig();
    config->load(settings);
  }
  settings.endGroup();
}

void Configs::reset() { clearConfigs(); }

void Configs::write(QDataStream& stream) const
{
  for (size_t index(0); index < configs_.count(); ++index)
  {
    configs_[index]->write(stream);
  }
}

void Configs::read(QDataStream& stream)
{
  for (size_t index(0); index < configs_.count(); ++index)
  {
    configs_[index]->read(stream);
  }
}

Configs& Configs::operator=(const Configs& config)
{
  while (configs_.count() < config.configs_.count())
  {
    addConfig();
  }
  while (configs_.count() > config.configs_.count())
  {
    removeConfig(configs_.count() - 1);
  }
  for (size_t index(0); index < configs_.count(); ++index)
  {
    *configs_[index] = *config.configs_[index];
  }
  return *this;
}

QString Configs::validate() const
{
  if (configs_.isEmpty())
  {
    return "Enter the system robots.";
  }
  QString validation;
  for (size_t i(0); i < configs_.count(); i++)
  {
    validation = configs_[i]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

void Configs::configChanged() { emit changed(); }

void Configs::configIdChanged(const QString& condig_id)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configIdChanged(index, condig_id);
    emit changed();
  }
}

void Configs::configAdded(const QString& full_name)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configAdded(index, full_name);
    emit changed();
  }
}

void Configs::configRemoved(const QString& full_name)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configRemoved(index, full_name);
    emit changed();
  }
}

void Configs::configCleared(const QString& full_name)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configCleared(index, full_name);
    emit changed();
  }
}

void Configs::configNameChanged(const QString& previous_name,
                                const QString& name)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configNameChanged(index, previous_name, name);
    emit changed();
  }
}

void Configs::configTypeChanged(const QString& name,
                                const QMetaType::Type& type)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configTypeChanged(index, name, type);
    emit changed();
  }
}

void Configs::configValueChanged(const QString& name, const QVariant& value)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configValueChanged(index, name, value);
    emit changed();
  }
}

void Configs::configDefaultValueChanged(const QString& name,
                                        const QVariant& default_value)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configDefaultValueChanged(index, name, default_value);
    emit changed();
  }
}

void Configs::configToolTipChanged(const QString& name, const QString& tool_tip)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configToolTipChanged(index, name, tool_tip);
    emit changed();
  }
}

void Configs::configDestroyed()
{
  Config* config = static_cast<Config*>(sender());
  int index(configs_.indexOf(config));
  if (index != -1)
  {
    QString config_id(config->getId());
    configs_.remove(index);
    emit removed(config_id);
    emit changed();
  }
}
}
}
}
