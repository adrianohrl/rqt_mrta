#include "rqt_mrta/idle_robots_config.h"

namespace rqt_mrta
{
IdleRobotsConfig::IdleRobotsConfig(QObject* parent)
    : AbstractTopicMonitorConfig(parent, "idle_robots")
{
}

IdleRobotsConfig::~IdleRobotsConfig() {}

void IdleRobotsConfig::reset()
{
  AbstractTopicMonitorConfig::reset();
  topic_config_->setName("/idle_robots");
  topic_config_->setType("mrta_msgs/Robot");
  topic_config_->setField("robot_id");
}

IdleRobotsConfig& IdleRobotsConfig::
operator=(const IdleRobotsConfig& config)
{
  AbstractTopicMonitorConfig::operator=(config);
  return *this;
}
}
