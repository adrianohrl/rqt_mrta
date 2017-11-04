#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
RqtMrtaArchitecture::RqtMrtaArchitecture(QObject* parent)
    : AbstractConfig(parent), architecture_(new Architecture(this)),
      widgets_(new Widgets(this))
{
  connect(architecture_, SIGNAL(changed()), this, SLOT(architectureChanged()));
  connect(widgets_, SIGNAL(changed()), this, SLOT(widgetsChanged()));
}

RqtMrtaArchitecture::~RqtMrtaArchitecture()
{
  if (architecture_)
  {
    delete architecture_;
    architecture_ = NULL;
  }
  if (widgets_)
  {
    delete widgets_;
    widgets_ = NULL;
  }
}

Architecture* RqtMrtaArchitecture::getArchitecture() const
{
  return architecture_;
}

Widgets* RqtMrtaArchitecture::getWidgets() const { return widgets_; }

void RqtMrtaArchitecture::save(QSettings& settings) const
{
  settings.beginGroup("rqt_mrta");
  architecture_->save(settings);
  widgets_->save(settings);
  settings.endGroup();
}

void RqtMrtaArchitecture::load(QSettings& settings)
{
  settings.beginGroup("rqt_mrta");
  architecture_->load(settings);
  widgets_->load(settings);
  settings.endGroup();
}

void RqtMrtaArchitecture::reset()
{
  architecture_->reset();
  widgets_->reset();
}

void RqtMrtaArchitecture::write(QDataStream& stream) const
{
  architecture_->write(stream);
  widgets_->write(stream);
}

void RqtMrtaArchitecture::read(QDataStream& stream)
{
  architecture_->read(stream);
  widgets_->read(stream);
}

RqtMrtaArchitecture& RqtMrtaArchitecture::
operator=(const RqtMrtaArchitecture& config)
{
  *architecture_ = *config.architecture_;
  *widgets_ = *config.widgets_;
}

void RqtMrtaArchitecture::architectureChanged() { emit changed(); }

void RqtMrtaArchitecture::widgetsChanged() { emit changed(); }
}
}
}
