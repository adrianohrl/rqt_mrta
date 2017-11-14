#include "rqt_mrta/config/architecture/architecture.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Architecture::Architecture(QObject* parent)
    : AbstractConfig(parent), allocations_(new Allocations(this)),
      launch_(new ArchitectureLaunch(this)), robots_(new Robots(this)),
      tasks_(new Tasks(this))
{
  connect(allocations_, SIGNAL(changed()), this, SLOT(allocationsChanged()));
  connect(launch_, SIGNAL(changed()), this, SLOT(launchChanged()));
  connect(robots_, SIGNAL(changed()), this, SLOT(robotsChanged()));
  connect(tasks_, SIGNAL(changed()), this, SLOT(tasksChanged()));
}

Architecture::~Architecture()
{
  ROS_INFO_STREAM("[~Architecture] before ...");
  if (allocations_)
  {
    delete allocations_;
    allocations_ = NULL;
  }
  if (launch_)
  {
    delete launch_;
    launch_ = NULL;
  }
  if (robots_)
  {
    delete robots_;
    robots_ = NULL;
  }
  if (tasks_)
  {
    delete tasks_;
    tasks_ = NULL;
  }
  ROS_INFO_STREAM("[~Architecture] after ...");
}

Allocations* Architecture::getAllocations() const { return allocations_; }

ArchitectureLaunch* Architecture::getLaunch() const { return launch_; }

QString Architecture::getName() const { return name_; }

Robots* Architecture::getRobots() const { return robots_; }

Tasks* Architecture::getTasks() const { return tasks_; }

void Architecture::setName(const QString& name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
  }
}

void Architecture::save(QSettings& settings) const
{
  settings.beginGroup("architecture");
  settings.setValue("name", name_);
  allocations_->save(settings);
  launch_->save(settings);
  robots_->save(settings);
  tasks_->save(settings);
  settings.endGroup();
}

void Architecture::load(QSettings& settings)
{
  settings.beginGroup("architecture");
  setName(settings.value("name").toString());
  allocations_->load(settings);
  launch_->load(settings);
  robots_->load(settings);
  tasks_->load(settings);
  settings.endGroup();
}

void Architecture::reset()
{
  setName("");
  allocations_->reset();
  launch_->reset();
  robots_->reset();
  tasks_->reset();
}

void Architecture::write(QDataStream& stream) const
{
  stream << name_;
  allocations_->write(stream);
  launch_->write(stream);
  robots_->write(stream);
  tasks_->write(stream);
}

void Architecture::read(QDataStream& stream)
{
  QString name;
  stream >> name;
  setName(name);
  allocations_->read(stream);
  launch_->read(stream);
  robots_->read(stream);
  tasks_->read(stream);
}

Architecture& Architecture::operator=(const Architecture& config)
{
  setName(config.name_);
  *allocations_ = *config.allocations_;
  *launch_ = *config.launch_;
  *robots_ = *config.robots_;
  *tasks_ = *config.tasks_;
  return *this;
}

void Architecture::allocationsChanged() { emit changed(); }

void Architecture::launchChanged() { emit changed(); }

void Architecture::robotsChanged() { emit changed(); }

void Architecture::tasksChanged() { emit changed(); }
}
}
}
