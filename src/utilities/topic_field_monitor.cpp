#include "utilities/topic_field_monitor.h"

namespace utilities
{
TopicFieldMonitor::TopicFieldMonitor(QObject *parent, const ros::NodeHandlePtr &nh, const QString &name, const size_t &queue_size, const QString &type, const QString &field)
  : QObject(parent), nh_(nh), name_(name), type_(type), field_(field), valid_(true), valid_field_(true)
{
  try
  {
    msg_type_.load(type_.toStdString());
    msg_definition_.load(type_.toStdString());
  }
  catch (const ros::Exception& exception)
  {
    msg_type_.clear();
    msg_definition_.clear();
    setValid(false, QString::fromStdString(exception.what()));
  }
  if (!msg_definition_.hasField(field_.toStdString()))
  {
    valid_field_ = false;
    setValid(false, "The given message does not have the " + field + " field.");
  }
  if (!isTopicRegistered() || matchTypes())
  {
    subscribe();
  }
  else
  {
    setValid(false, "The topic and message types do not match.");
  }
}

TopicFieldMonitor::~TopicFieldMonitor()
{
  unsubscribe();
  registry_.clear();
}

QString TopicFieldMonitor::getError() const
{
  return error_;
}

bool TopicFieldMonitor::isValid() const
{
  return valid_;
}

variant_topic_tools::BuiltinVariant TopicFieldMonitor::getCurrentFieldVariant() const
{
  return field_variant_;
}

ros::master::TopicInfo TopicFieldMonitor::getTopicInfo() const
{
  std::vector<ros::master::TopicInfo> topics;
  if (ros::master::getTopics(topics))
  {
    for (size_t i(0); i < topics.size(); ++i)
    {
      if (topics[i].name == name_.toStdString())
      {
        return topics[i];
      }
    }
  }
  return ros::master::TopicInfo();
}

bool TopicFieldMonitor::isTopicRegistered() const
{
  return !getTopicInfo().name.empty();
}

bool TopicFieldMonitor::isValidMessageType() const
{
  return !msg_type_.getDataType().empty();
}

bool TopicFieldMonitor::isValidField() const
{
  return valid_field_;
}

void TopicFieldMonitor::setValid(bool valid, const QString &error)
{
  if (valid != valid_)
  {
    valid_ = valid;
    error_ = error;
    emit validChanged(valid, error);
  }
}

bool TopicFieldMonitor::matchTypes() const
{
  ros::master::TopicInfo info(getTopicInfo());
  return !info.name.empty() && info.datatype == type_.toStdString();
}

void TopicFieldMonitor::subscribe()
{
  variant_topic_tools::MessageType type;
  subscriber_ = type.subscribe(*nh_, name_.toStdString(), queue_size_,
    boost::bind(&TopicFieldMonitor::callback, this, _1, _2));
  if (subscriber_)
  {
    emit subscribed(name_);
  }
}

void TopicFieldMonitor::unsubscribe()
{
  if (subscriber_)
  {
    subscriber_.shutdown();
    emit unsubscribed(name_);
  }
}

void TopicFieldMonitor::callback(const variant_topic_tools::MessageVariant &msg_variant, const ros::Time &receipt_timestamp)
{
  if (msg_variant.getType().getTypeInfo().name() != type_.toStdString().c_str())
  {
    setValid(false, "The topic and message types do not match.");
    unsubscribe();
    return;
  }
  receipt_timestamp_ = receipt_timestamp;
  field_variant_ = msg_variant.getMember(field_.toStdString());
  emit receivedMessageField(field_variant_, receipt_timestamp_);
}
}
