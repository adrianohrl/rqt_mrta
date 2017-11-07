#include "rqt_mrta/config/architecture/tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Tasks::Tasks(QObject* parent)
    : AbstractConfig(parent), incoming_tasks_(new IncomingTasks(this))
{
  connect(incoming_tasks_, SIGNAL(changed()), this,
          SLOT(incomingTasksChanged()));
}

Tasks::~Tasks()
{
  if (incoming_tasks_)
  {
    delete incoming_tasks_;
    incoming_tasks_ = NULL;
  }
}

IncomingTasks* Tasks::getIncomingTasks() const { return incoming_tasks_; }

void Tasks::save(QSettings& settings) const
{
  settings.beginGroup("tasks");
  incoming_tasks_->save(settings);
  settings.endGroup();
}

void Tasks::load(QSettings& settings)
{
  settings.beginGroup("tasks");
  incoming_tasks_->load(settings);
  settings.endGroup();
}

void Tasks::reset()
{
  incoming_tasks_->reset();
}

void Tasks::write(QDataStream& stream) const
{
  incoming_tasks_->write(stream);
}

void Tasks::read(QDataStream& stream)
{
  incoming_tasks_->read(stream);
}

Tasks& Tasks::operator=(const Tasks& config)
{
  *incoming_tasks_ = *config.incoming_tasks_;
  return *this;
}

void Tasks::incomingTasksChanged() { emit changed(); }
}
}
}
