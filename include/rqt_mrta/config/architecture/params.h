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

signals:
  void idChanged(const QString& name);
  void added(const QString& full_name);
  void removed(const QString& full_name);
  void cleared(const QString& full_name);
  void nameChanged(const QString& previous_name, const QString& name);
  void typeChanged(const QString& name, const QMetaType::Type& type);
  void valueChanged(const QString& name, const QVariant& value);
  void defaultValueChanged(const QString& name, const QVariant& default_value);
  void toolTipChanged(const QString& name, const QString& tool_tip);

protected:
  Params(const QString& group_name, Params* parent = NULL);

private:
  QVector<ParamInterface*> params_;

private slots:
  void paramNameChanged(const QString& previous_name, const QString& name);
  void paramTypeChanged(const QString& full_name, const QMetaType::Type& type);
  void paramValueChanged(const QString& full_name, const QVariant& value);
  void paramDefaultValueChanged(const QString& full_name,
                                const QVariant& default_value);
  void paramToolTipChanged(const QString& full_name, const QString& tool_tip);
  void paramAdded(const QString& full_name);
  void paramRemoved(const QString& full_name);
  void paramCleared(const QString& full_name);
  void paramDestroyed();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_H_
