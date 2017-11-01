#ifndef _RQT_MRTA_INCOMING_TASKS_CONFIG_H_
#define _RQT_MRTA_INCOMING_TASKS_CONFIG_H_

#include "rqt_mrta/abstract_topic_monitor_config.h"

namespace rqt_mrta
{
class TopicConfig;

class IncomingTasksConfig : public AbstractTopicMonitorConfig
{
  Q_OBJECT
public:
  IncomingTasksConfig(QObject* parent);
  virtual ~IncomingTasksConfig();
  virtual void reset();
  IncomingTasksConfig& operator=(const IncomingTasksConfig& config);
};
}

#endif // _RQT_MRTA_INCOMING_TASKS_CONFIG_H_
