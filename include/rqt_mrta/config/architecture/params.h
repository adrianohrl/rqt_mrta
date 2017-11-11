#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_H_

#include <QVector>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/param_interface.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Params : public ParamInterface
{
  Q_OBJECT
public:
  Params(Params* parent = NULL);
  virtual ~Params();
  ParamInterface* getParam(const QString& full_name) const;
  void addParam(ParamInterface* param);
  void removeParam(const QString& full_name);
  void clearParams();
  bool contains(const QString& full_name) const;
  size_t count() const;
  bool isEmpty() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Params& operator=(const Params& config);
  ParamInterface* clone() const;
  QString validate() const;
  static QStringList sortGroups(const QStringList& groups);

protected:
  Params(const QString& group_name, Params* parent = NULL);

private:
  QVector<ParamInterface*> params_;

private slots:
  void paramDestroyed();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_H_
