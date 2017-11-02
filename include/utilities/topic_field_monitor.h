#ifndef _UTILITIES_TOPIC_FIELD_MONITOR_H_
#define _UTILITIES_TOPIC_FIELD_MONITOR_H_

#include <QObject>
#include <QVariant>
#include <variant_topic_tools/Subscriber.h>

namespace utilities
{
class TopicFieldMonitor : public QObject
{
  Q_OBJECT
public:
  TopicFieldMonitor(QObject* parent, const ros::NodeHandlePtr& nh,
                    const QString& name, const size_t& queue_size,
                    const QString& type, const QString& field);
  virtual ~TopicFieldMonitor();
  QString getError() const;
  bool isTopicRegistered() const;
  bool isValidMessageType() const;
  bool isValidField() const;
  void setValid(bool valid, const QString& error);
  bool isValid() const;
  variant_topic_tools::BuiltinVariant getCurrentFieldVariant() const;

signals:
  void validChanged(bool valid, const QString& error);
  void subscribed(const QString& topic_name);
  void unsubscribed(const QString& topic_name);
  void
  receivedMessageField(const variant_topic_tools::BuiltinVariant& field_variant,
                       const ros::Time& receipt_timestamp);

private:  
  bool valid_field_;
  bool valid_;
  QString name_;
  size_t queue_size_;
  QString type_;
  QString field_;
  QString error_;
  ros::NodeHandlePtr nh_;
  variant_topic_tools::Subscriber subscriber_;
  variant_topic_tools::DataTypeRegistry registry_;
  variant_topic_tools::MessageType msg_type_;
  variant_topic_tools::MessageDefinition msg_definition_;
  ros::Time receipt_timestamp_;
  variant_topic_tools::BuiltinVariant field_variant_;
  bool matchTypes() const;
  ros::master::TopicInfo getTopicInfo() const;
  void subscribe();
  void unsubscribe();
  void callback(const variant_topic_tools::MessageVariant& variant,
                const ros::Time& receipt_timestamp);
};
}

#endif // _UTILITIES_TOPIC_FIELD_MONITOR_H_
