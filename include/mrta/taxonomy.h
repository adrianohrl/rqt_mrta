#ifndef _MRTA_TAXONOMY_H_
#define _MRTA_TAXONOMY_H_

#include <QString>

namespace mrta
{
class Taxonomy
{
public:
  enum AllocationType
  {
    UNKNOWN_ALLOCATION_TYPE,
    INSTANTANEOUS_ASSIGNMENT,
    TIME_EXTENDED_ASSIGNMENT
  };
  enum RobotType
  {
    UNKNOWN_ROBOT_TYPE,
    SINGLE_TASK,
    MULTI_TASK
  };
  enum TaskType
  {
    UNKNOWN_TASK_TYPE,
    SINGLE_ROBOT,
    MULTI_ROBOT
  };
  static AllocationType getAllocationType(const QString& type);
  static RobotType getRobotType(const QString& type);
  static TaskType getTaskType(const QString& type);
  static QString toQString(const AllocationType& type);
  static QString toQString(const RobotType& type);
  static QString toQString(const TaskType& type);
  static std::string toString(const AllocationType& type);
  static std::string toString(const RobotType& type);
  static std::string toString(const TaskType& type);
  static const char* toCString(const AllocationType& type);
  static const char* toCString(const RobotType& type);
  static const char* toCString(const TaskType& type);
};
}

#endif // _MRTA_TAXONOMY_H_
