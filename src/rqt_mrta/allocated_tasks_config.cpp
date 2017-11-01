#include "rqt_mrta/allocated_tasks_config.h"

namespace rqt_mrta
{
AllocatedTasksConfig::AllocatedTasksConfig(QObject* parent)
    : AbstractTopicMonitorConfig(parent, "allocated_tasks")
{
}

AllocatedTasksConfig::~AllocatedTasksConfig() {}

void AllocatedTasksConfig::reset()
{
  AbstractTopicMonitorConfig::reset();
  topic_config_->setName("/allocated_tasks");
  topic_config_->setType("mrta_msgs/Task");
  topic_config_->setField("task_id");
}

AllocatedTasksConfig& AllocatedTasksConfig::
operator=(const AllocatedTasksConfig& config)
{
  AbstractTopicMonitorConfig::operator=(config);
  return *this;
}
}
