#include "rqt_mrta/config/architecture/widget.h"

#include <ros/console.h>

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Widget::Widget(QObject *parent)
  : AbstractConfig(parent)
{
}

Widget::~Widget()
{
}

QString Widget::getId() const
{
  return id_;
}

void Widget::setId(const QString &id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

void Widget::save(QSettings &settings) const
{
  settings.setValue("id", id_);
}

void Widget::load(QSettings &settings)
{
  setId(settings.value("id").toString());
  ROS_WARN_STREAM("[Widget] id: " << id_.toStdString());
}

void Widget::reset()
{
  setId("");
}

void Widget::write(QDataStream &stream) const
{
  stream << id_;
}

void Widget::read(QDataStream &stream)
{
  QString id;
  stream >> id;
  setId(id);
}

Widget &Widget::operator=(const Widget &config)
{
  setId(config.id_);
}
}
}
}
