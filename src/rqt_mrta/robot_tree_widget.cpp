#include "mrta/architecture.h"
#include "mrta/robot.h"
#include "mrta/task.h"
#include "rqt_mrta/robot_tree_widget.h"

#include <ros/console.h>

Q_DECLARE_METATYPE(mrta::Robot*)
Q_DECLARE_METATYPE(mrta::Task*)

namespace rqt_mrta
{
RobotTreeWidget::RobotTreeWidget(QWidget* parent) : QTreeWidget(parent)
{
  setColumnCount(1);
  headerItem()->setText(0, "");
  QList<mrta::Robot*> robots;
  mrta::Task* t1 = new mrta::Task();
  t1->setId("t1");
  mrta::Task* t2 = new mrta::Task();
  t2->setId("t2");
  mrta::Task* t3 = new mrta::Task();
  t3->setId("t3");
  mrta::Task* t4 = new mrta::Task();
  t4->setId("t4");
  mrta::Robot* r1 = new mrta::Robot();
  r1->setId("r1");
  r1->addTask(t1);
  r1->addTask(t2);
  robots.append(r1);
  mrta::Robot* r2 = new mrta::Robot();
  r2->setId("r2");
  r2->addTask(t1);
  r2->addTask(t3);
  r2->addTask(t4);
  robots.append(r2);
  mrta::Robot* r3 = new mrta::Robot();
  r3->setId("r3");
  r3->addTask(t2);
  r3->addTask(t4);
  robots.append(r3);
  setRobots(robots);
  connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
          this, SLOT(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)));
}

RobotTreeWidget::~RobotTreeWidget() {}

QList<mrta::Robot*> RobotTreeWidget::getRobots() const
{
  QTreeWidgetItem* root = invisibleRootItem();
  QList<mrta::Robot*> robots;
  for (size_t i(0); i < root->childCount(); i++)
  {
    QTreeWidgetItem* item = root->child(i);
    robots.append(item->data(0, Qt::UserRole).value<mrta::Robot*>());
  }
  return robots;
}

void RobotTreeWidget::setRobots(const QList<mrta::Robot*> robots)
{
  clear();
  blockSignals(true);
  for (size_t i(0); i < robots.count(); i++)
  {
    addRobot(robots[i]);
  }
  blockSignals(false);
}

QString RobotTreeWidget::getCurrentId() const
{
  return current_type_ == ROBOT
             ? current_.robot_->getId()
             : current_type_ == TASK ? current_.task_->getId() : "";
}

void RobotTreeWidget::setCurrentId(const QString& id)
{
  QString previous_id;
  if (current_type_ == ROBOT)
  {
    previous_id = current_.robot_->getId();
    current_.robot_->setId(id);
    emit robotIdChanged(previous_id, id);
  }
  else if (current_type_ == TASK)
  {
    previous_id = current_.task_->getId();
    current_.task_->setId(id);
    emit taskIdChanged(previous_id, id);
  }
}

void RobotTreeWidget::addRobot(mrta::Robot* robot)
{
  QTreeWidgetItem* root = invisibleRootItem();
  QTreeWidgetItem* item = new QTreeWidgetItem(root);
  item->setText(0, robot->getId());
  item->setData(0, Qt::UserRole, QVariant::fromValue<mrta::Robot*>(robot));
  root->addChild(item);
  for (size_t j(0); j < robot->count(); j++)
  {
    addTask(robot->getTask(j), item);
  }
}

void RobotTreeWidget::addTask(mrta::Task* task, QTreeWidgetItem* parent)
{
  QTreeWidgetItem* child = new QTreeWidgetItem(parent);
  child->setText(0, task->getId());
  child->setData(0, Qt::UserRole, QVariant::fromValue<mrta::Task*>(task));
  parent->addChild(child);
}

bool RobotTreeWidget::contains(const QString& task_id) const
{
  return getTask(task_id);
}

mrta::Task* RobotTreeWidget::getTask(const QString& task_id) const
{
  for (const_iterator it(tasks_.constBegin()); it != tasks_.constEnd(); it++)
  {
    mrta::Task* task = *it;
    if (task->getId() == task_id)
    {
      return task;
    }
  }
  return NULL;
}

void RobotTreeWidget::currentItemChanged(QTreeWidgetItem* current,
                                         QTreeWidgetItem* previous)
{
  if (!current)
  {
    current_type_ = NONE;
    current_.robot_ = NULL;
  }
  else if (current->parent() == invisibleRootItem())
  {
    current_type_ = ROBOT;
    // current_ = current->
  }
}
}
