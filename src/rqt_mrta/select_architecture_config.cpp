#include "rqt_mrta/allocated_tasks_config.h"
#include "rqt_mrta/select_architecture_config.h"
#include "rqt_mrta/busy_robots_config.h"
#include "rqt_mrta/idle_robots_config.h"
#include "rqt_mrta/incoming_tasks_config.h"

namespace rqt_mrta
{
SelectArchitectureConfig::SelectArchitectureConfig(QObject* parent)
    : AbstractConfig(parent),
      allocated_tasks_config_(new AllocatedTasksConfig(this)),
      busy_robots_config_(new BusyRobotsConfig(this)),
      idle_robots_config_(new IdleRobotsConfig(this)),
      incoming_tasks_config_(new IncomingTasksConfig(this))
{
  connect(allocated_tasks_config_, SIGNAL(changed()), this,
          SLOT(allocatedTasksConfigChanged()));
  connect(busy_robots_config_, SIGNAL(changed()), this, SLOT(busyRobotsConfigChanged()));
  connect(idle_robots_config_, SIGNAL(changed()), this, SLOT(idleRobotsConfigChanged()));
  connect(incoming_tasks_config_, SIGNAL(changed()), this, SLOT(incomingTasksConfigChanged()));
}

SelectArchitectureConfig::~SelectArchitectureConfig()
{
  if (allocated_tasks_config_)
  {
    delete allocated_tasks_config_;
    allocated_tasks_config_ = NULL;
  }
  if (busy_robots_config_)
  {
    delete busy_robots_config_;
    busy_robots_config_ = NULL;
  }
  if (idle_robots_config_)
  {
    delete idle_robots_config_;
    idle_robots_config_ = NULL;
  }
  if (incoming_tasks_config_)
  {
    delete incoming_tasks_config_;
    incoming_tasks_config_ = NULL;
  }
}

AbstractTopicMonitorConfig*
SelectArchitectureConfig::getAllocatedTasksConfig() const
{
  return allocated_tasks_config_;
}

AbstractTopicMonitorConfig* SelectArchitectureConfig::getBusyRobotsConfig() const
{
  return busy_robots_config_;
}

AbstractTopicMonitorConfig* SelectArchitectureConfig::getIdleRobotsConfig() const
{
  return idle_robots_config_;
}

AbstractTopicMonitorConfig* SelectArchitectureConfig::getIncomingTasksConfig() const
{
  return incoming_tasks_config_;
}

void SelectArchitectureConfig::save(QSettings& settings) const
{
  settings.beginGroup("architecture");
  settings.beginGroup("robots");
  settings.beginGroup("idle_robots");
  idle_robots_config_->save(settings);
  settings.endGroup();
  settings.beginGroup("busy_robots");
  busy_robots_config_->save(settings);
  settings.endGroup();
  settings.endGroup();
  settings.beginGroup("tasks");
  settings.beginGroup("incoming_tasks");
  incoming_tasks_config_->save(settings);
  settings.endGroup();
  settings.endGroup();
  settings.beginGroup("allocations");
  settings.beginGroup("allocated_tasks");
  allocated_tasks_config_->save(settings);
  settings.endGroup();
  settings.endGroup();
  settings.endGroup();
}

void SelectArchitectureConfig::load(QSettings& settings)
{
  settings.beginGroup("robots");
  idle_robots_config_->load(settings);
  busy_robots_config_->load(settings);
  settings.endGroup();
  settings.beginGroup("tasks");
  incoming_tasks_config_->load(settings);
  settings.endGroup();
  settings.beginGroup("allocations");
  allocated_tasks_config_->load(settings);
  settings.endGroup();
}

void SelectArchitectureConfig::reset()
{
  allocated_tasks_config_->reset();
  busy_robots_config_->reset();
  idle_robots_config_->reset();
  incoming_tasks_config_->reset();
}

void SelectArchitectureConfig::write(QDataStream& stream) const
{
  allocated_tasks_config_->write(stream);
  busy_robots_config_->write(stream);
  idle_robots_config_->write(stream);
  incoming_tasks_config_->write(stream);
}

void SelectArchitectureConfig::read(QDataStream& stream)
{
  allocated_tasks_config_->read(stream);
  busy_robots_config_->read(stream);
  idle_robots_config_->read(stream);
  incoming_tasks_config_->read(stream);
}

SelectArchitectureConfig& SelectArchitectureConfig::
operator=(const SelectArchitectureConfig& config)
{
  *allocated_tasks_config_ = *config.allocated_tasks_config_;
  *busy_robots_config_ = *config.busy_robots_config_;
  *idle_robots_config_ = *config.idle_robots_config_;
  *incoming_tasks_config_ = *config.incoming_tasks_config_;
  return *this;
}

void SelectArchitectureConfig::allocatedTasksConfigChanged()
{
  emit changed();
}

void SelectArchitectureConfig::busyRobotsConfigChanged() { emit changed(); }

void SelectArchitectureConfig::idleRobotsConfigChanged() { emit changed(); }

void SelectArchitectureConfig::incomingTasksConfigChanged()
{
  emit changed();
}
}
