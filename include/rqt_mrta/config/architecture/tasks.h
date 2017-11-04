#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_TASKS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_TASKS_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class IncomingTasks;

class Tasks : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Tasks(QObject* parent = NULL);
  virtual ~Tasks();
  IncomingTasks* getIncomingTasks() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Tasks& operator=(const Tasks& config);

private:
  IncomingTasks* incoming_tasks_;

private slots:
  void incomingTasksChanged();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_TASKS_H_
