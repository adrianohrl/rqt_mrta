#ifndef _MRTA_ARCHITECTURE_CONFIG_H_
#define _MRTA_ARCHITECTURE_CONFIG_H_

#include "utilities/abstract_config.h"
#include "mrta/taxonomy.h"

namespace mrta
{
class ArchitectureConfig : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  ArchitectureConfig(QObject* parent = NULL);
  virtual ~ArchitectureConfig();
  bool belongs(const Taxonomy::AllocationType& allocation_type,
               const Taxonomy::RobotType& robot_type,
               const Taxonomy::TaskType& task_type) const;
  virtual void save(QSettings& settings) const;
  virtual void load(QSettings& settings);
  virtual void reset();
  virtual void write(QDataStream& stream) const;
  virtual void read(QDataStream& stream);

private:
  QString name_;
  Taxonomy::AllocationType allocation_type_;
  Taxonomy::RobotType robot_type_;
  Taxonomy::TaskType task_type_;
};
}

#endif // _MRTA_ARCHITECTURE_CONFIG_H_
