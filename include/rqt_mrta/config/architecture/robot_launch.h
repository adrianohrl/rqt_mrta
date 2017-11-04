#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOT_LAUNCH_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOT_LAUNCH_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class RobotLaunch : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  RobotLaunch(QObject* parent = NULL);
  virtual ~RobotLaunch();
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RobotLaunch& operator=(const RobotLaunch& config);
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOT_LAUNCH_H_
