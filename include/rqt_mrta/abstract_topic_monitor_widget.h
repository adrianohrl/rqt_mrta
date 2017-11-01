#ifndef _RQT_MRTA_ABSTRACT_TOPIC_MONITOR_WIDGET_H_
#define _RQT_MRTA_ABSTRACT_TOPIC_MONITOR_WIDGET_H_

#include <QWidget>

namespace Ui
{
class AbstractTopicMonitorWidget;
}

namespace rqt_mrta
{
class AbstractTopicMonitorConfig;

class AbstractTopicMonitorWidget : public QWidget
{
Q_OBJECT
public:
  AbstractTopicMonitorWidget(QWidget* parent);
  virtual ~AbstractTopicMonitorWidget();
  void setGroupBoxTitle(const QString& group_box_title);
  AbstractTopicMonitorConfig* getConfig() const;
  void setConfig(AbstractTopicMonitorConfig* config);

private:
  Ui::AbstractTopicMonitorWidget* ui_;
  AbstractTopicMonitorConfig* config_;

private slots:
  void configTopicNameChanged(const QString& name);
  void configTopicTypeChanged(const QString& type);
  void configTopicFieldChanged(const QString& field);
};
}

#endif // _RQT_MRTA_ABSTRACT_TOPIC_MONITOR_WIDGET_H_
