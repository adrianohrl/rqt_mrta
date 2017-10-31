#ifndef _RQT_MRTA_XML_SETTINGS_H_
#define _RQT_MRTA_XML_SETTINGS_H_

#include <QIODevice>
#include <QSettings>

namespace rqt_mrta
{
class XmlSettings {
public:
  static const QSettings::Format format;
  static bool read(QIODevice& device, QSettings::SettingsMap& map);
  static bool write(QIODevice& device, const QSettings::SettingsMap& map);
};
}

#endif // _RQT_MRTA_XML_SETTINGS_H_
