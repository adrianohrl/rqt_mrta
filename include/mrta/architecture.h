#ifndef _MRTA_ARCHITECTURE_H_
#define _MRTA_ARCHITECTURE_H_

#include <QObject>
#include "mrta/taxonomy.h"

namespace mrta
{

class ArchitectureConfig;

class Architecture : public QObject
{
  Q_OBJECT
public:
  Architecture(QObject* parent, const QString& package,
               const QString& configFilePath);
  virtual ~Architecture();
  bool belongs(const Taxonomy::AllocationType& allocation_type,
               const Taxonomy::RobotType& robot_type,
               const Taxonomy::TaskType& task_type) const;
  QString toString() const;
  bool operator==(const QString& package) const;
  bool operator==(const Architecture& architecture) const;

private:
  ArchitectureConfig* config_;
  const QString package_;
  QString name_;
};
}

#endif // _MRTA_ARCHITECTURE_H_
