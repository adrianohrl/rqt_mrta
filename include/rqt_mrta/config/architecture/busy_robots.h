#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_BUSY_ROBOTS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_BUSY_ROBOTS_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Topic;

class BusyRobots : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  BusyRobots(QObject* parent = NULL);
  virtual ~BusyRobots();
  Topic* getTopic() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  BusyRobots& operator=(const BusyRobots& config);

private:
  Topic* topic_;

private slots:
  void topicChanged();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_BUSY_ROBOTS_H_
