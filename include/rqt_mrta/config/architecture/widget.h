#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGET_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGET_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Widget : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Widget(QObject* parent = NULL);
  virtual ~Widget();
  QString getId() const;
  void setId(const QString& name);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Widget& operator=(const Widget& config);

signals:
  void idChanged(const QString &id);

private:
  QString id_;
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGET_H_
