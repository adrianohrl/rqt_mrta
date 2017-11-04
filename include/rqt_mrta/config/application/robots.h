#ifndef _RQT_MRTA_APPLICATION_CONFIG_ROBOTS_H_
#define _RQT_MRTA_APPLICATION_CONFIG_ROBOTS_H_

#include <QVector>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/application/robot.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robot;

class Robots : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Robots(QObject* parent = NULL);
  virtual ~Robots();
  size_t count() const;
  Robot* getRobot(size_t index) const;
  Robot* addRobot();
  void removeRobot(Robot* robot);
  void removeRobot(size_t index);
  void clearRobots();
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Robots& operator=(const Robots& config);

signals:
  void robotAdded(size_t index);
  void robotRemoved(size_t index);
  void robotsCleared();
  void robotChanged(size_t index);

private:
  typedef QVector<Robot*>::iterator iterator;
  typedef QVector<Robot*>::const_iterator const_iterator;
  QVector<Robot*> robots_;

private slots:
  void robotChanged();
  void robotDestroyed();
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_ROBOTS_H_
