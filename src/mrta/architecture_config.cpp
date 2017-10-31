#include "mrta/architecture_config.h"

namespace mrta
{
ArchitectureConfig::ArchitectureConfig(QObject* parent)
    : allocation_type_(Taxonomy::UNKNOWN_ALLOCATION_TYPE),
      robot_type_(Taxonomy::UNKNOWN_ROBOT_TYPE),
      task_type_(Taxonomy::UNKNOWN_TASK_TYPE)
{
}

ArchitectureConfig::~ArchitectureConfig() {}

bool ArchitectureConfig::belongs(
    const Taxonomy::AllocationType& allocation_type,
    const Taxonomy::RobotType& robot_type,
    const Taxonomy::TaskType& task_type) const
{
  return (allocation_type_ == allocation_type ||
          allocation_type == Taxonomy::UNKNOWN_ALLOCATION_TYPE) &&
         (robot_type_ == robot_type ||
          robot_type == Taxonomy::UNKNOWN_ROBOT_TYPE) &&
         (task_type_ == task_type || task_type == Taxonomy::UNKNOWN_TASK_TYPE);
}

void ArchitectureConfig::save(QSettings& settings) const {}

void ArchitectureConfig::load(QSettings& settings)
{
  name_ = settings.value("name").toString();
  allocation_type_ = Taxonomy::getAllocationType(
      settings.value("allocations/type", "").toString());
  robot_type_ =
      Taxonomy::getRobotType(settings.value("robots/type", "").toString());
  task_type_ =
      Taxonomy::getTaskType(settings.value("tasks/type", "").toString());
}

void ArchitectureConfig::reset() {}

void ArchitectureConfig::write(QDataStream& stream) const {}

void ArchitectureConfig::read(QDataStream& stream) {}
}
