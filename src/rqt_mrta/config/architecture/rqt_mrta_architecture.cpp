#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/config/architecture/architecture.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
RqtMrtaArchitecture::RqtMrtaArchitecture(QObject *parent)
  : AbstractConfig(parent), architecture_(new Architecture(this))
{
  connect(architecture_, SIGNAL(changed()), this, SLOT(architectureChanged()));
}

RqtMrtaArchitecture::~RqtMrtaArchitecture()
{
  if (architecture_)
  {
    delete architecture_;
    architecture_ = NULL;
  }
}

Architecture *RqtMrtaArchitecture::getArchitecture() const
{
  return architecture_;
}

void RqtMrtaArchitecture::save(QSettings &settings) const
{
  settings.beginGroup("rqt_mrta");
  architecture_->save(settings);
  settings.endGroup();
}

void RqtMrtaArchitecture::load(QSettings &settings)
{
  settings.beginGroup("rqt_mrta");
  architecture_->load(settings);
  settings.endGroup();
}

void RqtMrtaArchitecture::reset()
{
  architecture_->reset();
}

void RqtMrtaArchitecture::write(QDataStream &stream) const
{
  architecture_->write(stream);
}

void RqtMrtaArchitecture::read(QDataStream &stream)
{
  architecture_->read(stream);
}

RqtMrtaArchitecture &RqtMrtaArchitecture::operator=(const RqtMrtaArchitecture &config)
{
  *architecture_ = *config.architecture_;
}

void RqtMrtaArchitecture::architectureChanged()
{
  emit changed();
}
}
}
}
