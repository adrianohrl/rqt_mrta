#include "mrta/architecture.h"
#include "mrta/architecture_config.h"
#include <QFileInfo>
#include <QSettings>
#include <ros/console.h>
#include "utilities/xml_settings.h"

namespace mrta
{
Architecture::Architecture(QObject* parent, const QString& package,
                           const QString& configFilePath)
    : QObject(parent), config_(new ArchitectureConfig(this)), package_(package)
{
  if (!configFilePath.isEmpty())
  {
    QFileInfo fileInfo(configFilePath);
    if (fileInfo.isReadable())
    {
      QSettings settings(configFilePath, utilities::XmlSettings::format);
      if (settings.status() == QSettings::NoError)
      {
        settings.beginGroup("rqt_mrta");
        config_->load(settings);
        settings.endGroup();
      }
    }
  }
}

Architecture::~Architecture() {}

bool Architecture::belongs(const Taxonomy::AllocationType& allocation_type,
                           const Taxonomy::RobotType& robot_type,
                           const Taxonomy::TaskType& task_type) const
{
  return config_->belongs(allocation_type, robot_type, task_type);
}

QString Architecture::toString() const { return package_; }

bool Architecture::operator==(const QString& package) const
{
  return package_ == package;
}

bool Architecture::operator==(const Architecture& architecture) const
{
  return package_ == architecture.package_;
}
}
