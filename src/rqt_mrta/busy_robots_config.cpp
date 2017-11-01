#include "rqt_mrta/busy_robots_config.h"

namespace rqt_mrta
{
BusyRobotsConfig::BusyRobotsConfig(QObject* parent)
    : AbstractTopicMonitorConfig(parent, "busy_robots")
{
}

BusyRobotsConfig::~BusyRobotsConfig() {}

void BusyRobotsConfig::reset()
{
  AbstractTopicMonitorConfig::reset();
  topic_config_->setName("/busy_robots");
  topic_config_->setType("mrta_msgs/Robot");
  topic_config_->setField("robot_id");
}

BusyRobotsConfig& BusyRobotsConfig::operator=(const BusyRobotsConfig& config)
{
  AbstractTopicMonitorConfig::operator=(config);
  return *this;
}
}
