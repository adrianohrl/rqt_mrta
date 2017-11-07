#include "rqt_mrta/config/architecture/allocations.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Allocations::Allocations(QObject* parent)
    : AbstractConfig(parent), allocated_tasks_(new AllocatedTasks(this))
{
  connect(allocated_tasks_, SIGNAL(changed()), this,
          SLOT(allocatedTasksChanged()));
}

Allocations::~Allocations()
{
  if (allocated_tasks_)
  {
    delete allocated_tasks_;
    allocated_tasks_ = NULL;
  }
}

AllocatedTasks* Allocations::getAllocatedTasks() const
{
  return allocated_tasks_;
}

void Allocations::save(QSettings& settings) const
{
  settings.beginGroup("allocations");
  allocated_tasks_->save(settings);
  settings.endGroup();
}

void Allocations::load(QSettings& settings)
{
  settings.beginGroup("allocations");
  allocated_tasks_->load(settings);
  settings.endGroup();
}

void Allocations::reset() { allocated_tasks_->reset(); }

void Allocations::write(QDataStream& stream) const
{
  allocated_tasks_->write(stream);
}

void Allocations::read(QDataStream& stream) { allocated_tasks_->read(stream); }

Allocations& Allocations::operator=(const Allocations& config)
{
  *allocated_tasks_ = *config.allocated_tasks_;
  return *this;
}

void Allocations::allocatedTasksChanged() { emit changed(); }
}
}
}
