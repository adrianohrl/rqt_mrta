#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_CONFIG_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_CONFIG_H_

#include <QVector>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/param_interface.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class ParamInterface;

class Config : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Config(QObject* parent = NULL);
  virtual ~Config();
  QString getId() const;
  void setId(const QString& id);
  ParamInterface* getParam(const QString& full_name) const;
  void addParam(ParamInterface* param);
  void removeParam(const QString& full_name);
  void clearParams();
  void clearParams(const QString& full_name);
  bool contains(const QString& full_name) const;
  size_t count() const;
  size_t count(const QString& full_name) const;
  bool isEmpty() const;
  bool isEmpty(const QString& full_name) const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Config& operator=(const Config& config);
  QString validate() const;

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

private:
  QString id_;
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

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_CONFIG_H_
