#ifndef _RQT_MRTA_IDLE_ROBOTS_CONFIG_H_
#define _RQT_MRTA_IDLE_ROBOTS_CONFIG_H_

#include "rqt_mrta/abstract_topic_monitor_config.h"

namespace rqt_mrta
{
class IdleRobotsConfig : public AbstractTopicMonitorConfig
{
  Q_OBJECT
public:
  IdleRobotsConfig(QObject* parent);
  virtual ~IdleRobotsConfig();
  virtual void reset();
  IdleRobotsConfig& operator=(const IdleRobotsConfig& config);
};
}

#endif // _RQT_MRTA_IDLE_ROBOTS_CONFIG_H_
