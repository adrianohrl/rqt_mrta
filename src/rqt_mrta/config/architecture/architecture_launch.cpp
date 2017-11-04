#include "rqt_mrta/config/architecture/architecture_launch.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
ArchitectureLaunch::ArchitectureLaunch(QObject *parent)
  : AbstractConfig(parent)
{
}

ArchitectureLaunch::~ArchitectureLaunch()
{
}

void ArchitectureLaunch::save(QSettings &settings) const
{
  settings.beginGroup("architecture_launch");
  settings.endGroup();
}

void ArchitectureLaunch::load(QSettings &settings)
{
  settings.beginGroup("architecture_launch");
  settings.endGroup();
}

void ArchitectureLaunch::reset()
{
}

void ArchitectureLaunch::write(QDataStream &stream) const
{

}

void ArchitectureLaunch::read(QDataStream &stream)
{

}

ArchitectureLaunch &ArchitectureLaunch::operator=(const ArchitectureLaunch &config)
{
}
}
}
}
