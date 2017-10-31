#ifndef _UTILITIES_XML_SETTINGS_H_
#define _UTILITIES_XML_SETTINGS_H_

#include <QIODevice>
#include <QSettings>

namespace utilities
{
class XmlSettings {
public:
  static const QSettings::Format format;
  static bool read(QIODevice& device, QSettings::SettingsMap& map);
  static bool write(QIODevice& device, const QSettings::SettingsMap& map);
};
}

#endif // _UTILITIES_XML_SETTINGS_H_
