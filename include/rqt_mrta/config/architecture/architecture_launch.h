#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_ARCHITECTURE_LAUNCH_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_ARCHITECTURE_LAUNCH_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class ArchitectureLaunch : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  ArchitectureLaunch(QObject* parent = NULL);
  virtual ~ArchitectureLaunch();
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  ArchitectureLaunch& operator=(const ArchitectureLaunch& config);
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_ARCHITECTURE_LAUNCH_H_
