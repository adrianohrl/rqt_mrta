#ifndef _RQT_MRTA_APPLICATION_CONFIG_ROBOT_H_
#define _RQT_MRTA_APPLICATION_CONFIG_ROBOT_H_

#include <QMap>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/application/tasks.h"

namespace rqt_mrta
{
namespace config
{
class Config;
class Param;
class ParamInterface;
class Params;
class ParamsArray;

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
  Config* getConfig() const;
  void setConfig(Config* config);
  void setId(const QString& name);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Robot& operator=(const Robot& config);
  QString validate() const;

signals:
  void idChanged(const QString& id);
  void taskIdChanged(size_t task_index, const QString& task_id);
  void added(size_t task_index);
  void removed(const QString& task_id);
  void cleared();

private:
  typedef QMap<Param*, ParamsArray*> ArrayMap;
  typedef ArrayMap::iterator iterator;
  typedef ArrayMap::const_iterator const_iterator;
  QString id_;
  Tasks* tasks_;
  Config* config_;
  ArrayMap arrays_;
  void findArrays(Params *parent);
  void clearArrays();

private slots:
  void taskChanged(size_t task_index, const QString& task_id);
  void taskAdded(size_t task_index);
  void taskRemoved(const QString& task_id);
  void tasksCleared();
  void arraySizeChanged(const QString& full_name, const QVariant& value);
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_ROBOT_H_
