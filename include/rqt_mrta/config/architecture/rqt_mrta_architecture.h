#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Architecture;

class RqtMrtaArchitecture : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  RqtMrtaArchitecture(QObject* parent = NULL);
  virtual ~RqtMrtaArchitecture();
  Architecture* getArchitecture() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RqtMrtaArchitecture& operator=(const RqtMrtaArchitecture& config);

private:
  Architecture* architecture_;

private slots:
  void architectureChanged();
};
}
}

typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_H_
