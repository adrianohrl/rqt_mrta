#ifndef _MRTA_ROBOT_H_
#define _MRTA_ROBOT_H_

#include <QList>
#include <QObject>

namespace mrta
{
class Task;

class Robot : public QObject
{
  Q_OBJECT
public:
  typedef QList<Task*>::iterator iterator;
  typedef QList<Task*>::const_iterator const_iterator;
  Robot(QObject* object = NULL);
  Robot(const Robot& robot);
  virtual ~Robot();
  QString getId() const;
  void setId(const QString& id);
  size_t count() const;
  Task* getTask(int index) const;
  void addTask(Task* task);
  void removeTask(Task* task);
  void clearTasks();
  Robot& operator=(const Robot& robot);

signals:
  void changed();
  void idChanged();
  void taskAdded(Task* task);
  void taskRemoved(Task* task);

private:
  QString id_;
  QList<Task*> tasks_;

private slots:
  void tasksChanged();
};
}

#endif // _MRTA_ROBOT_H_
