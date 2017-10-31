#ifndef _UTILITIES_CONFIG_H_
#define _UTILITIES_CONFIG_H_

#include <QDataStream>
#include <QObject>
#include <QSettings>

namespace utilities
{
class Config : public QObject
{
  Q_OBJECT
public:
  Config(QObject* parent = NULL);
  virtual ~Config();
  virtual void save(QSettings& settings) const = 0;
  virtual void load(QSettings& settings) = 0;
  virtual void reset() = 0;
  virtual void write(QDataStream& stream) const = 0;
  virtual void read(QDataStream& stream) = 0;

signals:
  void changed();
};

QDataStream& operator<<(QDataStream& stream, const Config& config);
QDataStream& operator>>(QDataStream& stream, Config& config);
}

#endif // _UTILITIES_CONFIG_H_
