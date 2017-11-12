#ifndef _MRTA_ROBOT_H_
#define _MRTA_ROBOT_H_

#include <QVector>
#include <QObject>

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robot;
}

namespace architecture
{
class Robots;
class BusyRobots;
class IdleRobots;
}
}
}

namespace mrta
{
class Task;

class Robot : public QObject
{
  Q_OBJECT
public:
  typedef QList<Task*>::iterator iterator;
  typedef QList<Task*>::const_iterator const_iterator;
  typedef rqt_mrta::config::application::Robot Config;
  typedef rqt_mrta::config::architecture::Robots TopicsConfig;
  typedef rqt_mrta::config::architecture::BusyRobots BusyTopicConfig;
  typedef rqt_mrta::config::architecture::IdleRobots IdleTopicConfig;
  enum State
  {
    Idle,
    Busy,
    Offline
  };
  Robot(QObject* parent = NULL);
  Robot(QObject* parent, Config* config, TopicsConfig* topics_config);
  Robot(const Robot& robot);
  virtual ~Robot();
  Config* getConfig() const;
  BusyTopicConfig* getBusyTopicConfig() const;
  IdleTopicConfig* getIdleTopicConfig() const;
  void setConfig(Config* config);
  void setBusyTopicConfig(BusyTopicConfig* config);
  void setIdleTopicConfig(IdleTopicConfig* config);
  QString getId() const;
  State getState() const;
  void setState(State state);
  size_t count() const;
  Task* getTask(int index) const;
  void addTask(Task* task);
  void removeTask(Task* task);
  void clearTasks();
  Robot& operator=(const Robot& robot);

public slots:
  void setId(const QString& id);

signals:
  void changed();
  void idChanged(const QString& id);
  void stateChanged(State state);
  void idle();
  void busy();
  void offline();
  void added(size_t index);
  void removed(const QString& task_id);
  void taskChanged();
  void taskIdChanged(const QString& task_id);

private:
  QString id_;
  State state_;
  QVector<Task*> tasks_;
  Config* config_;
  BusyTopicConfig* busy_config_;
  IdleTopicConfig* idle_config_;

private slots:
  void configDestroyed();
  void busyTopicConfigDestroyed();
  void idleTopicConfigDestroyed();
  void taskDestroyed();
};
}

#endif // _MRTA_ROBOT_H_
