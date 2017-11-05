#include "mrta/architecture.h"
#include <rospack/rospack.h>
#include "rqt_mrta/architecture_combo_box.h"

namespace rqt_mrta
{
ArchitectureComboBox::ArchitectureComboBox(QWidget* parent)
    : QComboBox(parent), current_architecture_(NULL),
      allocation_type_(mrta::Taxonomy::UNKNOWN_ALLOCATION_TYPE),
      robot_type_(mrta::Taxonomy::UNKNOWN_ROBOT_TYPE),
      task_type_(mrta::Taxonomy::UNKNOWN_TASK_TYPE)
{
  load();
  filter();
  connect(this, SIGNAL(currentIndexChanged(int)), this,
          SLOT(currentArchitectureChanged(int)));
}

ArchitectureComboBox::~ArchitectureComboBox()
{
  current_architecture_ = NULL;
  for (iterator it(architectures_.begin()); it != architectures_.end(); it++)
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
  }
}

mrta::Architecture* ArchitectureComboBox::getCurrentArchitecture() const
{
  return current_architecture_;
}

void ArchitectureComboBox::setFilterAllocationType(
    const mrta::Taxonomy::AllocationType& allocation_type)
{
  if (allocation_type != allocation_type_)
  {
    allocation_type_ = allocation_type;
    filter();
    emit filterChanged();
    emit changed();
  }
}

void ArchitectureComboBox::setFilterRobotType(
    const mrta::Taxonomy::RobotType& robot_type)
{
  if (robot_type != robot_type_)
  {
    robot_type_ = robot_type;
    filter();
    emit filterChanged();
    emit changed();
  }
}

void ArchitectureComboBox::setFilterTaskType(
    const mrta::Taxonomy::TaskType& task_type)
{
  if (task_type != task_type_)
  {
    task_type_ = task_type;
    filter();
    emit filterChanged();
    emit changed();
  }
}

void ArchitectureComboBox::filter()
{
  setEnabled(false);
  clear();
  addItem("");
  int counter(0), index(-1);
  for (const_iterator it(architectures_.begin()); it != architectures_.end();
       it++)
  {
    mrta::Architecture* architecture = *it;
    if (architecture->belongs(allocation_type_, robot_type_, task_type_))
    {
      addItem(architecture->getPackage());
      counter++;
      if (current_architecture_ && *architecture == *current_architecture_)
      {
        index = counter;
      }
    }
  }
  if (count() > 1)
  {
    setEnabled(true);
  }
  if (index != -1)
  {
    setCurrentIndex(index);
  }
  else
  {
    current_architecture_ = NULL;
    emit unknownAchitecture();
  }
  emit filtered();
  emit changed();
}

void ArchitectureComboBox::load()
{
  std::vector<std::string> architectures;
  rospack::Rospack rp;
  rp.setQuiet(true);
  std::vector<std::string> search_path;
  rp.getSearchPathFromEnv(search_path);
  rp.crawl(search_path, true);
  architectures_.clear();
  if (rp.plugins("rqt_mrta", "architecture", "", architectures))
  {
    for (std::vector<std::string>::iterator it(architectures.begin());
         it != architectures.end(); it++)
    {
      size_t index(it->find(' '));
      QString package(QString::fromStdString(it->substr(0, index)));
      QString architecture_config_path(
          QString::fromStdString(it->substr(index + 1)));
      mrta::Architecture* mrta_architecture =
          new mrta::Architecture(this, package, architecture_config_path);
      architectures_.push_back(mrta_architecture);
    }
  }
}

void ArchitectureComboBox::currentArchitectureChanged(int index)
{
  current_architecture_ = index != 0 ? architectures_[index] : NULL;
  if (!current_architecture_)
  {
    emit unknownAchitecture();
  }
  emit currentArchitectureChanged(current_architecture_);
  emit changed();
}
}
