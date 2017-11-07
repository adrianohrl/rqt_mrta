#include <QMap>
#include <QList>
#include <QSharedPointer>
#include <QStringList>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>
#include "utilities/xml_settings.h"

#include <ros/console.h>

namespace utilities
{
const QSettings::Format XmlSettings::format =
    QSettings::registerFormat("xml", XmlSettings::read, XmlSettings::write);
const QString XmlSettings::GROUP_SEPARATOR = "/";
const QString XmlSettings::ATTRIBUTE_SEPARATOR = "@";

bool XmlSettings::read(QIODevice& device, QSettings::SettingsMap& map)
{
  QXmlStreamReader xml_reader(&device);
  QStringList groups;
  bool multiple_elements(false);
  while (!xml_reader.atEnd())
  {
    xml_reader.readNext();
    if (xml_reader.isStartElement())
    {
      groups.append(xml_reader.name().toString());
      multiple_elements = map.contains(groups.join(GROUP_SEPARATOR));
      QXmlStreamAttributes attributes(xml_reader.attributes());
      for (size_t i(0); i < attributes.count(); i++)
      {
        QString name(groups.join(GROUP_SEPARATOR) + ATTRIBUTE_SEPARATOR +
                     attributes[i].name().toString());
        if (!multiple_elements)
        {
          map[name] = attributes[i].value().toString();
        }
        else
        {
          QString values;
          if (!map.contains(name))
          {
            for (size_t j(0); j < i; j++)
            {
              values += ",";
            }
            values = "[" + values + "]";
          }
          values = map[name].toString();
          int count(values.count());
          QStringRef values_ref(&values, values[0] == '[' ? 1 : 0,
                                count - (values[count - 1] == ']' ? 2 : 0));
          map[name] = "[" + values_ref.toString() + "," +
                      xml_reader.text().toString() + "]";
        }
      }
    }
    else if (xml_reader.isCharacters() && !xml_reader.isWhitespace())
    {
      if (!multiple_elements)
      {
        map[groups.join(GROUP_SEPARATOR)] = xml_reader.text().toString();
      }
      else
      {
        QString values(map[groups.join(GROUP_SEPARATOR)].toString());
        int count(values.count());
        QStringRef values_ref(&values, values[0] == '[' ? 1 : 0,
                              count - (values[count - 1] == ']' ? 2 : 0));
        map[groups.join(GROUP_SEPARATOR)] = "[" + values_ref.toString() + "," +
                                            xml_reader.text().toString() + "]";
      }
    }
    else if (xml_reader.isEndElement())
    {
      groups.removeLast();
    }
  }
  return !xml_reader.hasError();
}

bool XmlSettings::write(QIODevice& device, const QSettings::SettingsMap& map)
{
  struct NestedMap;
  typedef QSharedPointer<NestedMap> NestedQSharedPointer;
  struct NestedMap : QMap<QString, NestedQSharedPointer>
  {
  };
  NestedQSharedPointer nested_map(new NestedMap());
  for (QSettings::SettingsMap::const_iterator it(map.begin()); it != map.end();
       ++it)
  {
    NestedQSharedPointer current_map(nested_map);
    QStringList groups(it.key().split(GROUP_SEPARATOR));
    for (QStringList::const_iterator jt(groups.begin()); jt != groups.end();
         ++jt)
    {
      NestedMap::iterator kt(current_map->find(*jt));
      if (kt == current_map->end())
      {
        kt = current_map->insert(*jt, NestedQSharedPointer(new NestedMap()));
        current_map = kt.value();
      }
      else
      {
        current_map = kt.value();
      }
    }
  }
  QXmlStreamWriter xml_writer(&device);
  xml_writer.setAutoFormatting(true);
  xml_writer.writeStartDocument();
  QStringList groups;
  QList<NestedQSharedPointer> nested_maps;
  QList<NestedMap::iterator> nested_map_iterators;
  nested_maps.append(nested_map);
  nested_map_iterators.append(nested_map->begin());
  while (!nested_maps.isEmpty())
  {
    NestedQSharedPointer current_map(nested_maps.last());
    NestedMap::iterator it(nested_map_iterators.last());
    if (it != current_map->end())
    {
      QStringList start_element(it.key().split('@'));
      xml_writer.writeStartElement(start_element[0]);
      if (start_element.count() > 1)
      {
        ROS_ERROR_STREAM("[XmlSettings::write] start_element.count() > 1 : "
                         << start_element.count()
                         << ", att[1]: " << start_element[1].toStdString());
      }
      groups.append(start_element[0]);
      nested_maps.append(it.value());
      nested_map_iterators.append(it.value()->begin());
    }
    else
    {
      if (current_map->isEmpty())
      {
        xml_writer.writeCharacters(map[groups.join("/")].toString());
      }
      xml_writer.writeEndElement();
      if (!groups.isEmpty())
      {
        groups.removeLast();
      }
      nested_maps.removeLast();
      nested_map_iterators.removeLast();
      if (!nested_maps.isEmpty())
      {
        ++nested_map_iterators.last();
      }
    }
  }
  xml_writer.writeEndDocument();
  return true;
}
}
