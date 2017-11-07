#ifndef _UTILITIES_ROS_PACKAGE_CONFIG_H_
#define _UTILITIES_ROS_PACKAGE_CONFIG_H_

#include <QStringList>
#include <rospack/rospack.h>
#include "utilities/abstract_config.h"

namespace utilities
{
class RosPackage : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  RosPackage(QObject* parent = NULL);
  virtual ~RosPackage();
  QString getWorkspaceUrl() const;
  QString getName() const;
  QString getUrl() const;
  QString getVersion() const;
  QString getDescription() const;
  QString getAuthor() const;
  QString getAuthorEmail() const;
  QString getMaintainer() const;
  QString getMaintainerEmail() const;
  QString getLicense() const;
  QString getBuildtoolDepend() const;
  QStringList getBuildDepends() const;
  QStringList getRunDepends() const;
  QStringList getExports() const;
  void setWorkspaceUrl(const QString& url);
  void setName(const QString& name);
  void setUrl(const QString& url);
  void setVersion(const QString& version);
  void setDescription(const QString& description);
  void setAuthor(const QString& name);
  void setAuthorEmail(const QString& email);
  void setMaintainer(const QString& name);
  void setMaintainerEmail(const QString& email);
  void setLicense(const QString& license);
  void setBuildtoolDepend(const QString& depend);
  void setBuildDepends(const QStringList& depends);
  void setRunDepends(const QStringList& depends);
  void setExports(const QStringList& exports);
  size_t countBuildDepends() const;
  size_t countRunDepends() const;
  size_t countExports() const;
  QString getBuildDepend(size_t index) const;
  QString getRunDepend(size_t index) const;
  QString getExport(size_t index) const;
  void addBuildDepend(const QString& depend);
  void addRunDepend(const QString& depend);
  void addExport(const QString& exporrt);
  void removeBuildDepend(const QString& depend);
  void removeRunDepend(const QString& depend);
  void removeExport(const QString& exporrt);
  void clearBuildDepends();
  void clearRunDepends();
  void clearExports();
  QString validate() const;
  bool isValid() const;
  bool isValidPackageName() const;
  virtual bool createPackage();
  bool createManifest();
  virtual bool createCMakeLists();
  bool updateManifest();
  QString getManifestUrl();
  QString getCMakeListsUrl();
  bool catkinMake() const;
  bool workspaceExists() const;
  bool catkinInitWorkspace() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  virtual void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RosPackage& operator=(const RosPackage& config);

signals:
  void workspaceUrlChanged(const QString& url);
  void nameChanged(const QString& name);
  void urlChanged(const QString& url);
  void versionChanged(const QString& version);
  void descriptionChanged(const QString& description);
  void authorChanged(const QString& name);
  void authorEmailChanged(const QString& email);
  void maintainerChanged(const QString& name);
  void maintainerEmailChanged(const QString& email);
  void licenseChanged(const QString& license);
  void buildtoolDependChanged(const QString& depend);
  void buildDependsChanged(const QStringList& depends);
  void runDependsChanged(const QStringList& depends);
  void exportsChanged(const QStringList& exports);
  void buildDependAdded(const QString& depend);
  void runDependAdded(const QString& depend);
  void exportAdded(const QString& exporrt);
  void buildDependRemoved(const QString& depend);
  void runDependRemoved(const QString& depend);
  void exportRemoved(const QString& exporrt);
  void buildDependsCleared();
  void runDependsCleared();
  void exportsCleared();

protected:
  rospack::Rospack rp_;

protected:
  QString workspace_url_;
  QString name_;
  QString url_;
  QString version_;
  QString description_;
  QString author_;
  QString author_email_;
  QString maintainer_;
  QString maintainer_email_;
  QString license_;
  QString buildtool_depend_;
  QStringList build_depends_;
  QStringList run_depends_;
  QStringList exports_;

private slots:
  void setUrl();
};

class RosMetapackage : public RosPackage
{
  Q_OBJECT
public:
  RosMetapackage(QObject* parent = NULL);
  virtual ~RosMetapackage();
  virtual void reset();
  bool createCMakeLists();
};
}

#endif // _UTILITIES_ROS_PACKAGE_CONFIG_H_
