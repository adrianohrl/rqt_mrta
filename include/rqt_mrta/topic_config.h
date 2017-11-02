#ifndef _RQT_MRTA_TOPIC_CONFIG_H_
#define _RQT_MRTA_TOPIC_CONFIG_H_

#include <ros/duration.h>
#include <ros/node_handle.h>
#include "utilities/abstract_config.h"
#include "utilities/topic_field_monitor.h"

namespace rqt_mrta
{
class TopicConfig : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  TopicConfig(QObject* parent);
  virtual ~TopicConfig();
  QString getName() const;
  QString getType() const;
  QString getField() const;
  ros::Duration getTimeout() const;
  ros::Duration getHorizon() const;
  void setName(const QString& name);
  void setType(const QString& type);
  void setField(const QString& field);
  void setTimeout(const ros::Duration& timeout);
  void setHorizon(const ros::Duration& horizon);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  TopicConfig& operator=(const TopicConfig& config);

signals:
  void nameChanged(const QString& name);
  void typeChanged(const QString& type);
  void fieldChanged(const QString& field);
  void timeoutChanged(const ros::Duration& timeout);
  void horizonChanged(const ros::Duration& horizon);

private:
  QString name_;
  QString type_;
  QString field_;
  ros::Duration timeout_;
  ros::Duration horizon_;
  ros::NodeHandlePtr nh_;
  utilities::TopicFieldMonitor* monitor_;

private slots:
  void updateMonitor();
  void monitorValidChanged(bool valid, const QString& error);
  void receivedMessageField(const variant_topic_tools::BuiltinVariant& field_variant,
                            const ros::Time& receipt_timestamp);
};
}

#endif // _RQT_MRTA_TOPIC_CONFIG_H_
