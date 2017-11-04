#ifndef _RQT_MRTA_APPLICATION_CONFIG_TASKS_H_
#define _RQT_MRTA_APPLICATION_CONFIG_TASKS_H_

#include <QVector>
#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Task;

class Tasks : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Tasks(QObject* parent = NULL);
  virtual ~Tasks();
  size_t count() const;
  Task* getTask(size_t index) const;
  Task* addTask();
  void removeTask(Task* task);
  void removeTask(size_t index);
  void clearTasks();
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Tasks& operator=(const Tasks& config);

signals:
  void taskAdded(size_t index);
  void taskRemoved(size_t index);
  void tasksCleared();
  void taskChanged(size_t index);

private:
  typedef QVector<Task*>::iterator iterator;
  typedef QVector<Task*>::const_iterator const_iterator;
  QVector<Task*> tasks_;

private slots:
  void taskChanged();
  void taskDestroyed();
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_TASKS_H_
