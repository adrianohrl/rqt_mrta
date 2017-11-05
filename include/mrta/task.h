#ifndef _MRTA_TASK_H_
#define _MRTA_TASK_H_

#include <QObject>

namespace mrta
{
class Task : public QObject
{
  Q_OBJECT
public:
  Task(QObject *parent = NULL);
  Task(const Task& task);
  virtual ~Task();
  QString getId() const;
  void setId(const QString& id);
  Task& operator=(const Task& task);

signals:
  void changed();
  void idChanged();

private:
  QString id_;
};
}

#endif // _MRTA_TASK_H_
