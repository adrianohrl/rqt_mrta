#ifndef _MRTA_ARCHITECTURE_H_
#define _MRTA_ARCHITECTURE_H_

#include <QObject>

namespace mrta
{
namespace types
{
enum AllocationType
{
  INSTANTANEOUS_ASSIGNMENT,
  TIME_EXTENDED_ASSIGNMENT
};

enum RobotType
{
  SINGLE_TASK,
  MULTI_TASK
};

enum TaskType
{
  SINGLE_ROBOT,
  MULTI_ROBOT
};
}

typedef types::AllocationType AllocationType;
typedef types::RobotType RobotType;
typedef types::TaskType TaskType;

class Architecture : public QObject
{
Q_OBJECT
public:
  Architecture(const QString &configFilePath);
  virtual ~Architecture();

private:
  QString name_;
  AllocationType allocation_type_;
  RobotType robot_type_;
  TaskType task_type_;
};
}

#endif // _MRTA_ARCHITECTURE_H_
