#include "rqt_mrta/config/architecture/robot_launch.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
RobotLaunch::RobotLaunch(QObject *parent)
  : AbstractConfig(parent)
{
}

RobotLaunch::~RobotLaunch()
{
}

void RobotLaunch::save(QSettings &settings) const
{
  settings.beginGroup("architecture_launch");
  settings.endGroup();
}

void RobotLaunch::load(QSettings &settings)
{
  settings.beginGroup("architecture_launch");
  settings.endGroup();
}

void RobotLaunch::reset()
{
}

void RobotLaunch::write(QDataStream &stream) const
{
}

void RobotLaunch::read(QDataStream &stream)
{
}

RobotLaunch &RobotLaunch::operator=(const RobotLaunch &config)
{
  return *this;
}
}
}
}
