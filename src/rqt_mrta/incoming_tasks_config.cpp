#include "rqt_mrta/incoming_tasks_config.h"

namespace rqt_mrta
{
IncomingTasksConfig::IncomingTasksConfig(QObject* parent)
    : AbstractTopicMonitorConfig(parent, "incoming_tasks")
{
}

IncomingTasksConfig::~IncomingTasksConfig() {}

void IncomingTasksConfig::reset()
{
  AbstractTopicMonitorConfig::reset();
  topic_config_->setName("/incoming_tasks");
  topic_config_->setType("mrta_msgs/Task");
  topic_config_->setField("task_id");
}

IncomingTasksConfig& IncomingTasksConfig::
operator=(const IncomingTasksConfig& config)
{
  AbstractTopicMonitorConfig::operator=(config);
  return *this;
}
}
