#ifndef _RQT_MRTA_CONFIG_INCLUDE_H_
#define _RQT_MRTA_CONFIG_INCLUDE_H_

#include <QStringList>
#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
class Args;
class Include : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Include(QObject* parent = NULL);
  virtual ~Include();
  QString getFile() const;
  QString getConfig(size_t index);
  Args* getArgs() const;
  void setFile(const QString& file);
  void addConfig(const QString& id);
  void removeConfig(const QString& id);
  void clearConfigs();
  size_t count();
  bool contains(const QString& id) const;
  QString validate() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Include& operator=(const Include& config);
  QString toLaunch(const QString& prefix) const;

signals:
  void fileChanged(const QString &file);
  void added(size_t index);
  void removed(const QString& file);
  void cleared();

private:
  QString file_;
  QStringList configs_;
  Args *args_;
};
}
}

#endif // _RQT_MRTA_CONFIG_INCLUDE_H_
