#ifndef _RQT_MRTA_ABSTRACT_TOPIC_MONITOR_CONFIG_H_
#define _RQT_MRTA_ABSTRACT_TOPIC_MONITOR_CONFIG_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/topic_config.h"

namespace rqt_mrta
{
class TopicConfig;

class AbstractTopicMonitorConfig : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  AbstractTopicMonitorConfig(QObject* parent, const QString& group_name);
  virtual ~AbstractTopicMonitorConfig();
  TopicConfig* getTopicConfig() const;
  virtual void save(QSettings& settings) const;
  virtual void load(QSettings& settings);
  virtual void reset();
  virtual void write(QDataStream& stream) const;
  virtual void read(QDataStream& stream);
  virtual AbstractTopicMonitorConfig&
  operator=(const AbstractTopicMonitorConfig& config);

protected:
  const QString group_name_;
  TopicConfig* topic_config_;

private slots:
  void topicConfigChanged();
};
}

#endif // _RQT_MRTA_ABSTRACT_TOPIC_MONITOR_CONFIG_H_
