#include <QFileInfo>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "utilities/exception.h"
#include "utilities/xml_settings.h"

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
  reset();
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

void RqtMrtaArchitecture::save(const QString& url) const
{
  if (url.isEmpty())
  {
    return;
  }
  QSettings settings(url, utilities::XmlSettings::format);
  if (settings.isWritable())
  {
    settings.clear();
    save(settings);
    settings.sync();
    if (settings.status() == QSettings::NoError)
    {
      ROS_INFO_STREAM("Saved architecture configuration file ["
                      << url.toStdString() << "].");
    }
  }
}

void RqtMrtaArchitecture::save(QSettings& settings) const
{
  settings.beginGroup("rqt_mrta");
  architecture_->save(settings);
  widgets_->save(settings);
  settings.endGroup();
}

void RqtMrtaArchitecture::load(const QString& url)
{
  QFileInfo file_info(url);
  if (!file_info.isReadable())
  {
    return;
  }
  QSettings settings(url, utilities::XmlSettings::format);
  if (settings.status() == QSettings::NoError)
  {
    load(settings);
    ROS_INFO_STREAM("Loaded architecture configuration file ["
                    << url.toStdString() << "].");
  }
}

void RqtMrtaArchitecture::load(QSettings& settings)
{
  QString type(settings.value("rqt_mrta@type").toString());
  if (type.isEmpty())
  {
    throw utilities::Exception("The <rqt_mrta> tag in the input xml file must "
                               "have an attribute named 'type'.");
  }
  if (type != "application")
  {
    throw utilities::Exception("The 'type' attribute of the <rqt_mrta> tag in "
                               "the input xml file must be valued as "
                               "'architecture' to be loaded as an architecture "
                               "configuration file.");
  }
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
  return *this;
}

void RqtMrtaArchitecture::architectureChanged() { emit changed(); }

void RqtMrtaArchitecture::widgetsChanged() { emit changed(); }
}
}
}
