#include "mrta/taxonomy.h"

namespace mrta
{
Taxonomy::AllocationType Taxonomy::getAllocationType(const QString& type)
{
  if (type == "instantaneous_assignment" || type == "IA" || type == "ia" ||
      type == "INSTANTANEOUS_ASSIGNMENT" || type == "InstantaneousAssignment" ||
      type == "Instantaneous Assignment" ||
      type == "Instantaneous Assignment (IA)")
  {
    return Taxonomy::INSTANTANEOUS_ASSIGNMENT;
  }
  else if (type == "time_extended_assignment" || type == "TA" || type == "ta" ||
           type == "TIME_EXTENDED_ASSIGNMENT" ||
           type == "TimeExtendedAssignment" ||
           type == "Time-Extended Assignment" ||
           type == "Time-Extended Assignment (TA)")
  {
    return Taxonomy::TIME_EXTENDED_ASSIGNMENT;
  }
  return Taxonomy::UNKNOWN_ALLOCATION_TYPE;
}

Taxonomy::RobotType Taxonomy::getRobotType(const QString& type)
{
  if (type == "single_task" || type == "ST" || type == "st" ||
      type == "SINGLE_TASK" || type == "SingleTask" || type == "Single Task" ||
      type == "Single Task (ST)")
  {
    return Taxonomy::SINGLE_TASK;
  }
  else if (type == "multi_task" || type == "MT" || type == "mt" ||
           type == "MULTI_TASK" || type == "MultiTask" ||
           type == "Multi Task" || type == "Multi Task (MT)")
  {
    return Taxonomy::MULTI_TASK;
  }
  return Taxonomy::UNKNOWN_ROBOT_TYPE;
}

Taxonomy::TaskType Taxonomy::getTaskType(const QString& type)
{
  if (type == "single_robot" || type == "SR" || type == "sr" ||
      type == "SINGLE_ROBOT" || type == "SingleRobot" ||
      type == "Single Robot" || type == "Single Robot (SR)")
  {
    return Taxonomy::SINGLE_ROBOT;
  }
  else if (type == "multi_robot" || type == "MR" || type == "mr" ||
           type == "MULTI_ROBOT" || type == "MultiRobot" ||
           type == "Multi Robot" || type == "Multi Robot (MR)")
  {
    return Taxonomy::MULTI_ROBOT;
  }
  return Taxonomy::UNKNOWN_TASK_TYPE;
}

QString Taxonomy::toQString(const Taxonomy::AllocationType& type)
{
  return type == Taxonomy::INSTANTANEOUS_ASSIGNMENT
             ? "IA"
             : type == Taxonomy::TIME_EXTENDED_ASSIGNMENT ? "TA" : "";
}

QString Taxonomy::toQString(const Taxonomy::RobotType& type)
{
  return type == Taxonomy::SINGLE_TASK ? "ST" : type == Taxonomy::MULTI_TASK
                                                    ? "MT"
                                                    : "";
}

QString Taxonomy::toQString(const Taxonomy::TaskType& type)
{
  return type == Taxonomy::SINGLE_ROBOT ? "SR" : type == Taxonomy::MULTI_ROBOT
                                                     ? "MR"
                                                     : "";
}

std::string Taxonomy::toString(const Taxonomy::AllocationType& type)
{
  return Taxonomy::toQString(type).toStdString();
}

std::string Taxonomy::toString(const Taxonomy::RobotType& type)
{
  return Taxonomy::toQString(type).toStdString();
}

std::string Taxonomy::toString(const Taxonomy::TaskType& type)
{
  return Taxonomy::toQString(type).toStdString();
}

const char* Taxonomy::toCString(const Taxonomy::AllocationType& type)
{
  return Taxonomy::toString(type).c_str();
}

const char* Taxonomy::toCString(const Taxonomy::RobotType& type)
{
  return Taxonomy::toString(type).c_str();
}

const char* Taxonomy::toCString(const Taxonomy::TaskType& type)
{
  return Taxonomy::toString(type).c_str();
}
}
