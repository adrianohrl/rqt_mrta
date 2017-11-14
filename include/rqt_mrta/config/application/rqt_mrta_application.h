#ifndef _RQT_MRTA_APPLICATION_CONFIG_H_
#define _RQT_MRTA_APPLICATION_CONFIG_H_

#include "utilities/abstract_config.h"
#include <rospack/rospack.h>
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
  QString getPackageUrl() const;
  Application* getApplication() const;
  void setPackage(const QString& package);
  void save() const;
  void save(const QString& filename) const;
  void load(const QString& filename);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RqtMrtaApplication& operator=(const RqtMrtaApplication& config);

signals:
  void packageChanged(const QString &package);

private:
  QString package_;
  QString package_url_;
  Application* application_;
  rospack::Rospack rp_;
  void save(QSettings& settings) const;
  void load(QSettings& settings);

private slots:
  void applicationChanged();
  void setPackageUrl(const QString& url);
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_H_
