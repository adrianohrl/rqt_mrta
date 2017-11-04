#ifndef _RQT_MRTA_APPLICATION_CONFIG_H_
#define _RQT_MRTA_APPLICATION_CONFIG_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/application/application.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplication : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  RqtMrtaApplication(QObject* parent = NULL);
  virtual ~RqtMrtaApplication();
  QString getPackage() const;
  Application* getApplication() const;
  void setPackage(const QString& package);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RqtMrtaApplication& operator=(const RqtMrtaApplication& config);

signals:
  void packageChanged(const QString &package);

private:
  QString package_;
  Application* application_;

private slots:
  void applicationChanged();
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_H_
