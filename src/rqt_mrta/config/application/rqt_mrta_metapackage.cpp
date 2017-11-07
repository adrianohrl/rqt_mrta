#include <QObject>
#include <QDir>
#include "rqt_mrta/config/application/rqt_mrta_metapackage.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
RqtMrtaApplicationMetapackage::RqtMrtaApplicationMetapackage(QObject* parent)
    : RosMetapackage(parent)
{
  reset();
}

RqtMrtaApplicationMetapackage::~RqtMrtaApplicationMetapackage() {}

void RqtMrtaApplicationMetapackage::reset()
{
  RosMetapackage::reset();
  addRunDepend("rqt_mrta");
  addExport("<rqt_mrta application=\"{prefix}/rqt_mrta.xml\"/>");
}

bool RqtMrtaApplicationMetapackage::createPackage()
{
  addRunDepend("rqt_mrta");
  addExport("<rqt_mrta application=\"{prefix}/rqt_mrta.xml\"/>");
  if (RosPackage::createPackage())
  {
    QDir package_dir(getPackageUrl());
    package_dir.mkdir("config");
    package_dir.mkdir("launch");
  }
}
}
}
}
