#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_ALLOCATED_TASKS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_ALLOCATED_TASKS_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Topic;

class AllocatedTasks : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  AllocatedTasks(QObject* parent = NULL);
  virtual ~AllocatedTasks();
  Topic* getTopic() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  AllocatedTasks& operator=(const AllocatedTasks& config);

private:
  Topic* topic_;

private slots:
  void topicChanged();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_ALLOCATED_TASKS_H_
