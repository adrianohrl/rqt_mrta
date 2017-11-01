#include "rqt_mrta/abstract_topic_monitor_config.h"

namespace rqt_mrta
{
AbstractTopicMonitorConfig::AbstractTopicMonitorConfig(
    QObject* parent, const QString& group_name)
    : AbstractConfig(parent), group_name_(group_name),
      topic_config_(new TopicConfig(this))
{
  connect(topic_config_, SIGNAL(changed()), this, SLOT(topicConfigChanged()));
}

AbstractTopicMonitorConfig::~AbstractTopicMonitorConfig()
{
  if (topic_config_)
  {
    delete topic_config_;
    topic_config_ = NULL;
  }
}

TopicConfig* AbstractTopicMonitorConfig::getTopicConfig() const
{
  return topic_config_;
}

void AbstractTopicMonitorConfig::save(QSettings& settings) const
{
  settings.beginGroup(group_name_);
  topic_config_->save(settings);
  settings.endGroup();
}

void AbstractTopicMonitorConfig::load(QSettings& settings)
{
  settings.beginGroup(group_name_);
  topic_config_->load(settings);
  settings.endGroup();
}

void AbstractTopicMonitorConfig::reset() { topic_config_->reset(); }

void AbstractTopicMonitorConfig::write(QDataStream& stream) const
{
  topic_config_->write(stream);
}

void AbstractTopicMonitorConfig::read(QDataStream& stream)
{
  topic_config_->read(stream);
}

AbstractTopicMonitorConfig& AbstractTopicMonitorConfig::
operator=(const AbstractTopicMonitorConfig& config)
{
  *topic_config_ = *config.topic_config_;
}

void AbstractTopicMonitorConfig::topicConfigChanged() { emit changed(); }
}
