#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_ARCHITECTURE_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_ARCHITECTURE_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/allocations.h"
#include "rqt_mrta/config/architecture/architecture_launch.h"
#include "rqt_mrta/config/architecture/robots.h"
#include "rqt_mrta/config/architecture/tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Architecture : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Architecture(QObject* parent = NULL);
  virtual ~Architecture();
  Allocations* getAllocations() const;
  ArchitectureLaunch* getLaunch() const;
  Robots* getRobots() const;
  Tasks* getTasks() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Architecture& operator=(const Architecture& config);

private:
  Allocations* allocations_;
  ArchitectureLaunch* launch_;
  Robots* robots_;
  Tasks* tasks_;

private slots:
  void allocationsChanged();
  void launchChanged();
  void robotsChanged();
  void tasksChanged();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_ARCHITECTURE_H_
