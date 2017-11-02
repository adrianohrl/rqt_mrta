#include "rqt_mrta/topic_config.h"

namespace rqt_mrta
{
TopicConfig::TopicConfig(QObject* parent)
    : AbstractConfig(parent), nh_(new ros::NodeHandle("~")), monitor_(NULL)
{
  connect(this, SIGNAL(changed()), this, SLOT(updateMonitor()));
}

TopicConfig::~TopicConfig()
{
  if (monitor_)
  {
    delete monitor_;
    monitor_ = NULL;
  }
}

QString TopicConfig::getName() const { return name_; }

QString TopicConfig::getType() const { return type_; }

QString TopicConfig::getField() const { return field_; }

ros::Duration TopicConfig::getTimeout() const { return timeout_; }

ros::Duration TopicConfig::getHorizon() const { return horizon_; }

void TopicConfig::setName(const QString& name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
  }
}

void TopicConfig::setType(const QString& type)
{
  if (type != type_)
  {
    type_ = type;
    emit typeChanged(type);
    emit changed();
  }
}

void TopicConfig::setField(const QString& field)
{
  if (field != field_)
  {
    field_ = field;
    emit fieldChanged(field);
    emit changed();
  }
}

void TopicConfig::setTimeout(const ros::Duration& timeout)
{
  if (timeout != timeout_)
  {
    timeout_ = timeout;
    emit timeoutChanged(timeout);
    emit changed();
  }
}

void TopicConfig::setHorizon(const ros::Duration& horizon)
{
  if (horizon != horizon_)
  {
    horizon_ = horizon;
    emit horizonChanged(horizon);
    emit changed();
  }
}

void TopicConfig::save(QSettings& settings) const
{
  settings.beginGroup("topic");
  settings.setValue("name", name_);
  settings.setValue("type", type_);
  settings.setValue("field", field_);
  settings.setValue("timeout", timeout_.toSec());
  settings.setValue("horizon", horizon_.toSec());
  settings.endGroup();
}

void TopicConfig::load(QSettings& settings)
{
  settings.beginGroup("topic");
  setName(settings.value("name").toString());
  setType(settings.value("type").toString());
  setField(settings.value("field").toString());
  setTimeout(ros::Duration(settings.value("timeout", 2.0).toDouble()));
  setHorizon(ros::Duration(settings.value("horizon", 5.0).toDouble()));
  settings.endGroup();
}

void TopicConfig::reset()
{
  setTimeout(ros::Duration(2.0));
  setHorizon(ros::Duration(5.0));
}

void TopicConfig::write(QDataStream& stream) const
{
  stream << name_;
  stream << type_;
  stream << field_;
  stream << timeout_.toSec();
  stream << horizon_.toSec();
}

void TopicConfig::read(QDataStream& stream)
{
  QString name, type, field;
  double timeout, horizon;
  stream >> name;
  setName(name);
  stream >> type;
  setType(type);
  stream >> field;
  setField(field);
  stream >> timeout;
  setTimeout(ros::Duration(timeout));
  stream >> horizon;
  setHorizon(ros::Duration(horizon));
}

TopicConfig& TopicConfig::operator=(const TopicConfig& config)
{
  setName(config.name_);
  setType(config.type_);
  setField(config.field_);
  setTimeout(config.timeout_);
  setHorizon(config.horizon_);
}

void TopicConfig::updateMonitor()
{
  ROS_INFO("[TopicConfig] updating monitor ...");
  if (monitor_)
  {
    disconnect(monitor_, SIGNAL(validChanged(bool, const QString&)), this,
               SLOT(monitorValidChanged(bool, const QString&)));
    disconnect(
        monitor_,
        SIGNAL(receivedMessageField(const variant_topic_tools::BuiltinVariant&,
                                    const ros::Time&)),
        this,
        SLOT(receivedMessageField(const variant_topic_tools::BuiltinVariant&,
                                  const ros::Time&)));
    delete monitor_;
    monitor_ = NULL;
  }
  monitor_ =
      new utilities::TopicFieldMonitor(this, nh_, name_, 10, type_, field_);
  if (!monitor_->isValid())
  {
    ROS_ERROR_STREAM("[TopicFieldMonitor] invalid " << monitor_->getError().toStdString());
    delete monitor_;
    monitor_ = NULL;
    return;
  }
  connect(monitor_, SIGNAL(validChanged(bool, const QString&)), this,
          SLOT(monitorValidChanged(bool, const QString&)));
  connect(monitor_,
          SIGNAL(receivedMessageField(
              const variant_topic_tools::BuiltinVariant&, const ros::Time&)),
          this,
          SLOT(receivedMessageField(const variant_topic_tools::BuiltinVariant&,
                                    const ros::Time&)));
}

void TopicConfig::monitorValidChanged(bool valid, const QString& error)
{
  if (!valid)
  {
    ROS_ERROR_STREAM("[TopicFieldMonitor] " << error.toStdString());
  }
}

void TopicConfig::receivedMessageField(
    const variant_topic_tools::BuiltinVariant& field_variant,
    const ros::Time& receipt_timestamp)
{
  ROS_INFO_STREAM("[TopicConfig] received: "
                  << field_variant.getValue<std::string>() << " at "
                  << receipt_timestamp);
}
}
