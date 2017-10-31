#include "mrta/architecture.h"
#include <QFileInfo>
#include <QSettings>
#include "rqt_mrta/xml_settings.h"

namespace mrta
{
Architecture::Architecture(const QString& configFilePath)
{
  if (!configFilePath.isEmpty())
  {
    QFileInfo fileInfo(configFilePath);
    if (fileInfo.isReadable())
    {
      QSettings settings(configFilePath, rqt_mrta::XmlSettings::format);
      if (settings.status() == QSettings::NoError)
      {
        settings.beginGroup("rqt_mrta");
        config_->load(settings);
        settings.endGroup();
      }
    }
  }
}

Architecture::~Architecture()
{

}
}
