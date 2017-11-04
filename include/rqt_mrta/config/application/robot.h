#ifndef _RQT_MRTA_APPLICATION_CONFIG_ROBOT_H_
#define _RQT_MRTA_APPLICATION_CONFIG_ROBOT_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/application/tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robot : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Robot(QObject* parent = NULL);
  virtual ~Robot();
  QString getId() const;
  Tasks* getTasks() const;
  void setId(const QString& name);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Robot& operator=(const Robot& config);

signals:
  void idChanged(const QString &id);

private:
  QString id_;
  Tasks* tasks_;

private slots:
  void tasksChanged();
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_ROBOT_H_
