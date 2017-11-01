#ifndef _RQT_MRTA_BUSY_ROBOTS_CONFIG_H_
#define _RQT_MRTA_BUSY_ROBOTS_CONFIG_H_

#include "rqt_mrta/abstract_topic_monitor_config.h"

namespace rqt_mrta
{
class TopicConfig;

class BusyRobotsConfig : public AbstractTopicMonitorConfig
{
  Q_OBJECT
public:
  BusyRobotsConfig(QObject* parent);
  virtual ~BusyRobotsConfig();
  virtual void reset();
  BusyRobotsConfig& operator=(const BusyRobotsConfig& config);
};
}

#endif // _RQT_MRTA_BUSY_ROBOTS_CONFIG_H_
