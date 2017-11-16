#ifndef _RQT_MRTA_APPLICATION_METAPACKAGE_CONFIG_H_
#define _RQT_MRTA_APPLICATION_METAPACKAGE_CONFIG_H_

#include "utilities/ros_package.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplicationPackage : public utilities::RosMetapackage
{
  Q_OBJECT
public:
  RqtMrtaApplicationPackage(QObject* parent);
  virtual ~RqtMrtaApplicationPackage();
  virtual bool createPackage();
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_METAPACKAGE_CONFIG_H_
