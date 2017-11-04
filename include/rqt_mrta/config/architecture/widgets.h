#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGETS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGETS_H_

#include <QVector>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/widget.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Widgets : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Widgets(QObject* parent = NULL);
  virtual ~Widgets();
  size_t count() const;
  Widget* getWidget(size_t index) const;
  Widget* addWidget();
  void removeWidget(Widget* task);
  void removeWidget(size_t index);
  void clearWidgets();
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Widgets& operator=(const Widgets& config);

signals:
  void widgetAdded(size_t index);
  void widgetRemoved(size_t index);
  void widgetsCleared();
  void widgetChanged(size_t index);

private:
  typedef QVector<Widget*>::iterator iterator;
  typedef QVector<Widget*>::const_iterator const_iterator;
  QVector<Widget*> widgets_;

private slots:
  void widgetChanged();
  void widgetDestroyed();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGETS_H_
