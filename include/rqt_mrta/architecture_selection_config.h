#ifndef _RQT_MRTA_ARCHITECTURE_SELECTION_CONFIG_H_
#define _RQT_MRTA_ARCHITECTURE_SELECTION_CONFIG_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
class AbstractTopicMonitorConfig;
class AllocatedTasksConfig;
class BusyRobotsConfig;
class IdleRobotsConfig;
class IncomingTasksConfig;

class ArchitectureSelectionConfig : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  ArchitectureSelectionConfig(QObject* parent);
  virtual ~ArchitectureSelectionConfig();
  AbstractTopicMonitorConfig *getAllocatedTasksConfig() const;
  AbstractTopicMonitorConfig* getBusyRobotsConfig() const;
  AbstractTopicMonitorConfig* getIdleRobotsConfig() const;
  AbstractTopicMonitorConfig* getIncomingTasksConfig() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  ArchitectureSelectionConfig&
  operator=(const ArchitectureSelectionConfig& config);

private:
  AllocatedTasksConfig* allocated_tasks_config_;
  BusyRobotsConfig* busy_robots_config_;
  IdleRobotsConfig* idle_robots_config_;
  IncomingTasksConfig* incoming_tasks_config_;

private slots:
  void allocatedTasksConfigChanged();
  void busyRobotsConfigChanged();
  void idleRobotsConfigChanged();
  void incomingTasksConfigChanged();
};
}

#endif // _RQT_MRTA_ARCHITECTURE_SELECTION_CONFIG_H_
