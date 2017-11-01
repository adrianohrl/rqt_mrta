#include "mrta/architecture.h"
#include "mrta/architecture_config.h"
#include <QFileInfo>
#include <QSettings>
#include <ros/console.h>
#include "utilities/xml_settings.h"

namespace mrta
{
Architecture::Architecture(QObject* parent, const QString& package,
                           const QString& config_file_path)
    : QObject(parent), config_(new ArchitectureConfig(this)), package_(package),
      config_file_path_(config_file_path)
{
  if (!config_file_path.isEmpty())
  {
    QFileInfo file_info(config_file_path);
    if (file_info.isReadable())
    {
      QSettings settings(config_file_path, utilities::XmlSettings::format);
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

QString Architecture::getPackage() const { return package_; }

QString Architecture::getConfigFilePath() const { return config_file_path_; }

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
