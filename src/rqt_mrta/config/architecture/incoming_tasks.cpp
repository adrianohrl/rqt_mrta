#include "rqt_mrta/config/architecture/incoming_tasks.h"
#include "rqt_mrta/config/architecture/topic.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
IncomingTasks::IncomingTasks(QObject* parent)
    : AbstractConfig(parent), topic_(new Topic(this))
{
  connect(topic_, SIGNAL(changed()), this,
          SLOT(topicChanged()));
}

IncomingTasks::~IncomingTasks()
{
  if (topic_)
  {
    delete topic_;
    topic_ = NULL;
  }
}

Topic* IncomingTasks::getTopic() const
{
  return topic_;
}

void IncomingTasks::save(QSettings& settings) const
{
  settings.beginGroup("incoming_tasks");
  topic_->save(settings);
  settings.endGroup();
}

void IncomingTasks::load(QSettings& settings)
{
  settings.beginGroup("incoming_tasks");
  topic_->load(settings);
  settings.endGroup();
}

void IncomingTasks::reset()
{
  topic_->reset();
  topic_->setName("/incoming_tasks");
  topic_->setType("mrta_msgs/Task");
  topic_->setField("task_id");
}

void IncomingTasks::write(QDataStream& stream) const
{
  topic_->write(stream);
}

void IncomingTasks::read(QDataStream& stream)
{
  topic_->read(stream);
}

IncomingTasks& IncomingTasks::operator=(const IncomingTasks& config)
{
  *topic_ = *config.topic_;
}

void IncomingTasks::topicChanged() { emit changed(); }
}
}
}
