#ifndef _RQT_MRTA_APPLICATION_CONFIG_APPLICATION_H_
#define _RQT_MRTA_APPLICATION_CONFIG_APPLICATION_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robots;

class Application : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Application(QObject* parent = NULL);
  virtual ~Application();
  QString getName() const;
  QString getUrl() const;
  Robots* getRobots() const;
  void setName(const QString& name);
  void setUrl(const QString& url);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Application& operator=(const Application& config);

signals:
  void nameChanged(const QString &name);
  void urlChanged(const QString &url);

private:
  QString name_;
  QString url_;
  Robots* robots_;

private slots:
  void robotsChanged();
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_APPLICATION_H_
