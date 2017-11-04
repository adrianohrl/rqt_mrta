#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/architecture.h"
#include "rqt_mrta/config/architecture/widgets.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class RqtMrtaArchitecture : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  RqtMrtaArchitecture(QObject* parent = NULL);
  virtual ~RqtMrtaArchitecture();
  Architecture* getArchitecture() const;
  Widgets* getWidgets() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RqtMrtaArchitecture& operator=(const RqtMrtaArchitecture& config);

private:
  Architecture* architecture_;
  Widgets* widgets_;

private slots:
  void architectureChanged();
  void widgetsChanged();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_H_
