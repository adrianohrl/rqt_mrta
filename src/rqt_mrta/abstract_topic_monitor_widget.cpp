#include "rqt_mrta/abstract_topic_monitor_config.h"
#include "rqt_mrta/abstract_topic_monitor_widget.h"
#include "rqt_mrta/ui_abstract_topic_monitor_widget.h"

namespace rqt_mrta
{
AbstractTopicMonitorWidget::AbstractTopicMonitorWidget(QWidget* parent)
    : QWidget(parent), ui_(new Ui::AbstractTopicMonitorWidget()), config_(NULL)
{
  ui_->setupUi(this);
  connect(ui_->name_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(configTopicNameChanged(const QString&)));
  connect(ui_->type_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(configTopicTypeChanged(const QString&)));
  connect(ui_->field_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(configTopicFieldChanged(const QString&)));
}

AbstractTopicMonitorWidget::~AbstractTopicMonitorWidget()
{
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}

void AbstractTopicMonitorWidget::setGroupBoxTitle(
    const QString& group_box_title)
{
  ui_->group_box->setTitle(group_box_title);
}

AbstractTopicMonitorConfig* AbstractTopicMonitorWidget::getConfig() const
{
  return config_;
}

void AbstractTopicMonitorWidget::setConfig(AbstractTopicMonitorConfig* config)
{
  if (config_ != config)
  {
    if (config_)
    {
      disconnect(config_->getTopicConfig(), SIGNAL(nameChanged(const QString&)), this,
                 SLOT(configTopicNameChanged(const QString&)));
      disconnect(config_->getTopicConfig(), SIGNAL(typeChanged(const QString&)), this,
                 SLOT(configTopicTypeChanged(const QString&)));
      disconnect(config_->getTopicConfig(), SIGNAL(fieldChanged(const QString&)), this,
                 SLOT(configTopicFieldChanged(const QString&)));
    }
    config_ = config;
    if (config)
    {
      connect(config_->getTopicConfig(), SIGNAL(nameChanged(const QString&)), this,
              SLOT(configTopicNameChanged(const QString&)));
      connect(config_->getTopicConfig(), SIGNAL(typeChanged(const QString&)), this,
              SLOT(configTopicTypeChanged(const QString&)));
      connect(config_->getTopicConfig(), SIGNAL(fieldChanged(const QString&)), this,
              SLOT(configTopicFieldChanged(const QString&)));
    }
    configTopicNameChanged(config_->getTopicConfig()->getName());
    configTopicTypeChanged(config_->getTopicConfig()->getType());
    configTopicFieldChanged(config_->getTopicConfig()->getField());
  }
}

void AbstractTopicMonitorWidget::configTopicNameChanged(const QString& name)
{
  ui_->name_line_edit->setText(name);
}

void AbstractTopicMonitorWidget::configTopicTypeChanged(const QString& type)
{
  ui_->type_line_edit->setText(type);
}

void AbstractTopicMonitorWidget::configTopicFieldChanged(const QString& field)
{
  ui_->field_line_edit->setText(field);
}
}
