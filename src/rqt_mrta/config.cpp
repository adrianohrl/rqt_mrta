#include "rqt_mrta/config.h"

namespace rqt_mrta
{
Config::Config(QObject* parent) :
  QObject(parent)
{
}

Config::~Config()
{
}

QDataStream& operator<<(QDataStream& stream, const Config& config) {
  config.write(stream);
  return stream;
}

QDataStream& operator>>(QDataStream& stream, Config& config) {
  config.read(stream);
  return stream;
}
}
