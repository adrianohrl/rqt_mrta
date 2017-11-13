#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOTS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOTS_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/busy_robots.h"
#include "rqt_mrta/config/architecture/idle_robots.h"
#include "rqt_mrta/config/architecture/robot_launch.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Robots : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Robots(QObject* parent = NULL);
  virtual ~Robots();
  QString getType() const;
  BusyRobots* getBusyRobots() const;
  IdleRobots* getIdleRobots() const;
  RobotLaunch* getLaunch() const;
  void setType(const QString& type);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Robots& operator=(const Robots& config);

signals:
  void changed();
  void typeChanged();

private:
  QString type_;
  BusyRobots* busy_robots_;
  IdleRobots* idle_robots_;
  RobotLaunch* launch_;
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOTS_H_
