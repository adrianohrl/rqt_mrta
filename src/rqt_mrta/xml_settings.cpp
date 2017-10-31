#include <QMap>
#include <QList>
#include <QSharedPointer>
#include <QStringList>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>
#include "rqt_mrta/xml_settings.h"

namespace rqt_mrta
{
const QSettings::Format XmlSettings::format =
    QSettings::registerFormat("xml", XmlSettings::read, XmlSettings::write);

bool XmlSettings::read(QIODevice& device, QSettings::SettingsMap& map)
{
  QXmlStreamReader xmlReader(&device);
  QStringList groups;
  while (!xmlReader.atEnd())
  {
    xmlReader.readNext();
    if (xmlReader.isStartElement())
    {
      groups.append(xmlReader.name().toString());
    }
    else if (xmlReader.isCharacters() && !xmlReader.isWhitespace())
    {
      map[groups.join("/")] = xmlReader.text().toString();
    }
    else if (xmlReader.isEndElement())
    {
      groups.removeLast();
    }
  }
  return !xmlReader.hasError();
}

bool XmlSettings::write(QIODevice& device, const QSettings::SettingsMap& map)
{
  struct NestedMap;
  typedef QSharedPointer<NestedMap> NestedQSharedPointer;
  struct NestedMap : QMap<QString, NestedQSharedPointer>
  {
  };
  NestedQSharedPointer nestedMap(new NestedMap());
  for (QSettings::SettingsMap::const_iterator it(map.begin()); it != map.end();
       ++it)
  {
    NestedQSharedPointer currentMap(nestedMap);
    QStringList groups(it.key().split("/"));
    for (QStringList::const_iterator jt(groups.begin()); jt != groups.end();
         ++jt)
    {
      NestedMap::iterator kt(currentMap->find(*jt));
      if (kt == currentMap->end())
      {
        kt = currentMap->insert(*jt, NestedQSharedPointer(new NestedMap()));
        currentMap = kt.value();
      }
      else
      {
        currentMap = kt.value();
      }
    }
  }
  QXmlStreamWriter xmlWriter(&device);
  xmlWriter.setAutoFormatting(true);
  xmlWriter.writeStartDocument();
  QStringList groups;
  QList<NestedQSharedPointer> nestedMaps;
  QList<NestedMap::iterator> nestedMapIterators;
  nestedMaps.append(nestedMap);
  nestedMapIterators.append(nestedMap->begin());
  while (!nestedMaps.isEmpty())
  {
    NestedQSharedPointer currentMap(nestedMaps.last());
    NestedMap::iterator it(nestedMapIterators.last());
    if (it != currentMap->end())
    {
      xmlWriter.writeStartElement(it.key());
      groups.append(it.key());
      nestedMaps.append(it.value());
      nestedMapIterators.append(it.value()->begin());
    }
    else
    {
      if (currentMap->isEmpty())
      {
        xmlWriter.writeCharacters(map[groups.join("/")].toString());
      }
      xmlWriter.writeEndElement();
      if (!groups.isEmpty())
      {
        groups.removeLast();
      }
      nestedMaps.removeLast();
      nestedMapIterators.removeLast();
      if (!nestedMaps.isEmpty())
      {
        ++nestedMapIterators.last();
      }
    }
  }
  xmlWriter.writeEndDocument();
  return true;
}
}
