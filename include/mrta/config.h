#ifndef _RQT_MRTA_CONFIG_H_
#define _RQT_MRTA_CONFIG_H_

#include <QDataStream>
#include <QObject>
#include <QSettings>

namespace rqt_mrta {
class Config : public QObject
{
Q_OBJECT
public:
  Config(QObject* parent = NULL);
  ~Config();

  virtual void save(QSettings& settings) const = NULL;
  virtual void load(QSettings& settings) = NULL;
  virtual void reset() = NULL;

  virtual void write(QDataStream& stream) const = NULL;
  virtual void read(QDataStream& stream) = NULL;

signals:
  void changed();
};

QDataStream& operator<<(QDataStream& stream, const Config& config);
QDataStream& operator>>(QDataStream& stream, Config& config);
}

#endif // _RQT_MRTA_CONFIG_H_
