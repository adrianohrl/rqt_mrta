#include <QObject>
#include <QDir>
#include <ros/console.h>
#include "rqt_mrta/config/application/rqt_mrta_package.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
RqtMrtaApplicationPackage::RqtMrtaApplicationPackage(QObject* parent)
    : RosMetapackage(parent)
{
  reset();
}

RqtMrtaApplicationPackage::~RqtMrtaApplicationPackage() {}

bool RqtMrtaApplicationPackage::createPackage()
{
  addRunDepend("rqt_mrta");
  export_->add("rqt_mrta/@application", "{prefix}/rqt_mrta.xml");
  if (RosPackage::createPackage())
  {
    QDir package_dir(getUrl());
    package_dir.mkdir("config");
    package_dir.mkdir("launch");
  }
}
}
}
}
