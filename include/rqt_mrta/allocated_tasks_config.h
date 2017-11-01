#ifndef _RQT_MRTA_ALLOCATED_TASKS_CONFIG_H_
#define _RQT_MRTA_ALLOCATED_TASKS_CONFIG_H_

#include "rqt_mrta/abstract_topic_monitor_config.h"

namespace rqt_mrta
{
class AllocatedTasksConfig : public AbstractTopicMonitorConfig
{
  Q_OBJECT
public:
  AllocatedTasksConfig(QObject* parent);
  virtual ~AllocatedTasksConfig();
  virtual void reset();
  AllocatedTasksConfig& operator=(const AllocatedTasksConfig& config);
};
}

#endif // _RQT_MRTA_ALLOCATED_TASKS_CONFIG_H_
