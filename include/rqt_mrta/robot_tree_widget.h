#ifndef _RQT_MRTA_ROBOTS_TREE_WIDGET_H_
#define _RQT_MRTA_ROBOTS_TREE_WIDGET_H_

#include <QList>
#include <QTreeWidget>

namespace mrta
{
class Robot;
class Task;
}

namespace rqt_mrta
{
class RobotTreeWidget : public QTreeWidget
{
  Q_OBJECT
public:
  enum Type
  {
    NONE,
    ROBOT,
    TASK
  };
  RobotTreeWidget(QWidget* parent = NULL);
  virtual ~RobotTreeWidget();
  QList<mrta::Robot*> getRobots() const;
  void setRobots(const QList<mrta::Robot*> robots);
  QString getCurrentId() const;
  void setCurrentId(const QString& id);

signals:
  void robotIdChanged(const QString& previous_id, const QString& id);
  void taskIdChanged(const QString& previous_id, const QString& id);
  void robotSelected(mrta::Robot* robot);
  void taskSelected(mrta::Task* task);
  void selected(const QString& id);

private:
  typedef QList<mrta::Task*>::iterator iterator;
  typedef QList<mrta::Task*>::const_iterator const_iterator;
  union Selection
  {
    mrta::Robot* robot_;
    mrta::Task* task_;
  };
  Type current_type_;
  QList<mrta::Task*> tasks_;
  Selection current_;
  void addRobot(mrta::Robot *robot);
  void addTask(mrta::Task *task, QTreeWidgetItem* parent);
  bool contains(const QString& task_id) const;
  mrta::Task* getTask(const QString& task_id) const;

private slots:
  void currentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem*
    previous);
};
}

#endif // _RQT_MRTA_ROBOTS_TREE_WIDGET_H_
