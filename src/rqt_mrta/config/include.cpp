#include "rqt_mrta/config/include.h"
#include "rqt_mrta/config/args.h"

namespace rqt_mrta
{
namespace config
{
Include::Include(QObject* parent)
    : AbstractConfig(parent), args_(new Args(this))
{
  connect(args_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Include::~Include()
{
  if (args_)
  {
    delete args_;
    args_ = NULL;
  }
}

QString Include::getFile() const { return file_; }

QString Include::getConfig(size_t index) { return configs_[index]; }

Args* Include::getArgs() const { return args_; }

void Include::setFile(const QString& file)
{
  if (file != file_)
  {
    file_ = file;
    emit fileChanged(file);
    emit changed();
  }
}

void Include::addConfig(const QString& id)
{
  if (!configs_.contains(id))
  {
    configs_.append(id);
    emit added(configs_.count() - 1);
    emit changed();
  }
}

void Include::removeConfig(const QString& id)
{
  if (configs_.contains(id))
  {
    configs_.removeAll(id);
    emit removed(id);
    emit changed();
  }
}

void Include::clearConfigs()
{
  if (configs_.isEmpty())
  {
    configs_.clear();
    emit cleared();
    emit changed();
  }
}

size_t Include::count() { return configs_.count(); }

bool Include::contains(const QString& id) const
{
  return configs_.contains(id);
}

QString Include::validate() const
{
  if (file_.isEmpty())
  {
    return "Enter the file location to be included.";
  }
  return args_->validate();
}

void Include::save(QSettings& settings) const
{
  settings.setValue("file", file_);
  for (size_t index(0); index < configs_.count(); index++)
  {
    settings.setValue("config_" + QString::number(index), configs_[index]);
  }
  args_->save(settings);
}

void Include::load(QSettings& settings)
{
  setFile(settings.value("file").toString());
  clearConfigs();
  QStringList configs(settings.allKeys().filter("config_"));
  for (size_t index(0); index < configs.count(); index++)
  {
    addConfig(settings.value(configs[index]).toString());
  }
  args_->load(settings);
}

void Include::reset()
{
  setFile("");
  clearConfigs();
  args_->reset();
}

void Include::write(QDataStream& stream) const
{
  stream << file_;
  stream << configs_.count();
  for (size_t index(0); index < configs_.count(); index++)
  {
    stream << configs_[index];
  }
  args_->write(stream);
}

void Include::read(QDataStream& stream)
{
  QString file;
  quint64 count;
  QString config;
  stream >> file;
  setFile(file);
  stream >> count;
  clearConfigs();
  for (size_t index(0); index < count; index++)
  {
    stream >> config;
    addConfig(config);
  }
  args_->read(stream);
}

Include& Include::operator=(const Include& config)
{
  setFile(config.file_);
  clearConfigs();
  for (size_t index(0); index < config.configs_.count(); index++)
  {
    addConfig(config.configs_[index]);
  }
  *args_ = *config.args_;
  return *this;
}

QString Include::toLaunch(const QString &prefix) const
{
  QString launch;
  launch += prefix + "<include file=\"" + file_ + "\">\n";
  launch += args_->toLaunch(prefix);
  launch += prefix + "</include>\n";
  return launch;
}
}
}
