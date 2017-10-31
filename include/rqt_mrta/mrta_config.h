#ifndef _RQT_MRTA_MRTA_CONFIG_H_
#define _RQT_MRTA_MRTA_CONFIG_H_

#include "utilities/config.h"

namespace rqt_mrta
{
class MRTAConfig : public utilities::Config
{
Q_OBJECT
public:
  MRTAConfig(QObject* parent = NULL);
  virtual ~MRTAConfig();
  virtual void save(QSettings& settings) const;
  virtual void load(QSettings& settings);
  virtual void reset();
  virtual void write(QDataStream& stream) const;
  virtual void read(QDataStream& stream);
};
}

#endif // _RQT_MRTA_MRTA_CONFIG_H_
