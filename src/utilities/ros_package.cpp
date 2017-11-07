#include <QDir>
#include <QFileInfo>
#include <QTextStream>
#include <ros/console.h>
#include "utilities/ros_package.h"
#include "utilities/xml_settings.h"

namespace utilities
{
RosPackage::RosPackage(QObject* parent) : AbstractConfig(parent)
{
  rp_.setQuiet(true);
  std::vector<std::string> search_path;
  rp_.getSearchPathFromEnv(search_path);
  rp_.crawl(search_path, true);
}

RosPackage::~RosPackage() {}

QString RosPackage::getWorkspaceUrl() const { return workspace_url_; }

QString RosPackage::getName() const { return name_; }

QString RosPackage::getUrl() const { return url_; }

QString RosPackage::getVersion() const { return version_; }

QString RosPackage::getDescription() const { return description_; }

QString RosPackage::getAuthor() const { return author_; }

QString RosPackage::getAuthorEmail() const { return author_email_; }

QString RosPackage::getMaintainer() const { return maintainer_; }

QString RosPackage::getMaintainerEmail() const { return maintainer_email_; }

QString RosPackage::getLicense() const { return license_; }

QString RosPackage::getBuildtoolDepend() const { return buildtool_depend_; }

QStringList RosPackage::getBuildDepends() const { return build_depends_; }

QStringList RosPackage::getRunDepends() const { return run_depends_; }

QStringList RosPackage::getExports() const { return exports_; }

void RosPackage::setWorkspaceUrl(const QString& url)
{
  if (url != workspace_url_)
  {
    workspace_url_ = url;
    emit workspaceUrlChanged(url);
    emit changed();
  }
}

void RosPackage::setName(const QString& name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
    setUrl();
  }
}

void RosPackage::setUrl()
{
  std::string url;
  rp_.find(name_.toStdString(), url);
  setUrl(QString::fromStdString(url));
}

void RosPackage::setUrl(const QString& url)
{
  if (url != url_)
  {
    url_ = url;
    emit urlChanged(url);
    emit changed();
  }
}

void RosPackage::setVersion(const QString& version)
{
  if (version != version_)
  {
    version_ = version;
    emit versionChanged(version);
    emit changed();
  }
}

void RosPackage::setDescription(const QString& description)
{
  if (description != description_)
  {
    description_ = description;
    emit descriptionChanged(description);
    emit changed();
  }
}

void RosPackage::setAuthor(const QString& name)
{
  if (name != author_)
  {
    author_ = name;
    emit authorChanged(name);
    emit changed();
  }
}

void RosPackage::setAuthorEmail(const QString& email)
{
  if (email != author_email_)
  {
    author_email_ = email;
    emit authorEmailChanged(email);
    emit changed();
  }
}

void RosPackage::setMaintainer(const QString& name)
{
  if (name != maintainer_)
  {
    maintainer_ = name;
    emit maintainerChanged(name);
    emit changed();
  }
}

void RosPackage::setMaintainerEmail(const QString& email)
{
  if (email != maintainer_email_)
  {
    maintainer_email_ = email;
    emit maintainerEmailChanged(email);
    emit changed();
  }
}

void RosPackage::setLicense(const QString& license)
{
  if (license != license_)
  {
    license_ = license;
    emit licenseChanged(license);
    emit changed();
  }
}

void RosPackage::setBuildtoolDepend(const QString& depend)
{
  if (depend != buildtool_depend_)
  {
    buildtool_depend_ = depend;
    emit buildtoolDependChanged(depend);
    emit changed();
  }
}

void RosPackage::setBuildDepends(const QStringList& depends)
{
  if (depends != build_depends_)
  {
    build_depends_ = depends;
    emit buildDependsChanged(depends);
    emit changed();
  }
}

void RosPackage::setRunDepends(const QStringList& depends)
{
  if (depends != run_depends_)
  {
    run_depends_ = depends;
    emit runDependsChanged(depends);
    emit changed();
  }
}

void RosPackage::setExports(const QStringList& exports)
{
  if (exports != exports_)
  {
    exports_ = exports;
    emit exportsChanged(exports);
    emit changed();
  }
}

size_t RosPackage::countBuildDepends() const { return build_depends_.count(); }

size_t RosPackage::countRunDepends() const { return run_depends_.count(); }

size_t RosPackage::countExports() const { return exports_.count(); }

QString RosPackage::getBuildDepend(size_t index) const
{
  return build_depends_[index];
}

QString RosPackage::getRunDepend(size_t index) const
{
  return run_depends_[index];
}

QString RosPackage::getExport(size_t index) const { return exports_[index]; }

void RosPackage::addBuildDepend(const QString& depend)
{
  std::string package_path;
  rp_.find(depend.toStdString(), package_path);
  if (!build_depends_.contains(depend) && !package_path.empty())
    ;
  {
    build_depends_.append(depend);
    emit buildDependAdded(depend);
    emit changed();
  }
}

void RosPackage::addRunDepend(const QString& depend)
{
  std::string package_path;
  rp_.find(depend.toStdString(), package_path);
  if (!run_depends_.contains(depend) && !package_path.empty())
  {
    run_depends_.append(depend);
    emit runDependAdded(depend);
    emit changed();
  }
}

void RosPackage::addExport(const QString& exporrt)
{
  if (!exports_.contains(exporrt))
  {
    exports_.append(exporrt);
    emit exportAdded(exporrt);
    emit changed();
  }
}

void RosPackage::removeBuildDepend(const QString& depend)
{
  if (build_depends_.contains(depend))
  {
    build_depends_.removeAll(depend);
    emit buildDependRemoved(depend);
    emit changed();
  }
}

void RosPackage::removeRunDepend(const QString& depend)
{
  if (run_depends_.contains(depend))
  {
    run_depends_.removeAll(depend);
    emit runDependRemoved(depend);
    emit changed();
  }
}

void RosPackage::removeExport(const QString& exporrt)
{
  if (exports_.contains(exporrt))
  {
    exports_.removeAll(exporrt);
    emit exportRemoved(exporrt);
    emit changed();
  }
}

void RosPackage::clearBuildDepends()
{
  if (!build_depends_.isEmpty())
  {
    build_depends_.clear();
    emit buildDependsCleared();
    emit changed();
  }
}

void RosPackage::clearRunDepends()
{
  if (!run_depends_.isEmpty())
  {
    run_depends_.clear();
    emit runDependsCleared();
    emit changed();
  }
}

void RosPackage::clearExports()
{
  if (!exports_.isEmpty())
  {
    exports_.clear();
    emit exportsCleared();
    emit changed();
  }
}

QString RosPackage::validate() const
{
  if (name_.isEmpty())
  {
    return "The package name must not be empty.";
  }
  else if (name_.contains(' '))
  {
    return "The package name must not contain space characters.";
  }
  else if (version_.isEmpty())
  {
    return "The package version must be defined.";
  }
  else if (description_.isEmpty())
  {
    return "The package description must be given.";
  }
  else if (maintainer_.isEmpty())
  {
    return "The package maintainer must be given.";
  }
  else if (license_.isEmpty())
  {
    return "The package license must be defined.";
  }
  return "";
}

bool RosPackage::isValid() const
{
  return !name_.isEmpty() && !name_.contains(' ') && url_.isEmpty() &&
         !version_.isEmpty() && !description_.isEmpty() &&
         !maintainer_.isEmpty() && !license_.isEmpty();
}

bool RosPackage::isValidPackageName() const
{
  return !name_.isEmpty() && !name_.contains(' ');
}

bool RosPackage::createPackage()
{
  if (!getPackageUrl().isEmpty())
  {
    ROS_WARN_STREAM("The given package [" << name_.toStdString()
                                          << "] already exists.");
    return false;
  }
  QDir workspace_dir(workspace_url_);
  if (workspaceExists() || catkinInitWorkspace())
  {
    QDir package_dir(workspace_url_ + "/src");
    package_dir.mkdir(name_);
    return createManifest() && createCMakeLists() && catkinMake();
  }
  return false;
}

bool RosPackage::createManifest()
{
  QString error_message(validate());
  if (!error_message.isEmpty())
  {
    ROS_ERROR("%s", error_message.toStdString().c_str());
    return false;
  }
  QString package_url(getPackageUrl());
  if (package_url.isEmpty())
  {
    ROS_ERROR_STREAM("The given package [" << name_.toStdString()
                                           << "] doest not exist.");
    return false;
  }
  QString file_url(package_url + "/package.xml");
  QFileInfo file_info(file_url);
  if (file_info.exists())
  {
    ROS_ERROR_STREAM("The given package ["
                     << name_.toStdString()
                     << "] already has its package.xml file.");
    return false;
  }
  QSettings settings(package_url, utilities::XmlSettings::format);
  if (settings.isWritable())
  {
    settings.clear();
    save(settings);
    settings.sync();
    if (settings.status() == QSettings::NoError)
    {
      ROS_INFO_STREAM("Created the " << name_.toStdString()
                                     << " manifest file.");
      return true;
    }
  }
  ROS_ERROR_STREAM("Unable to create the " << name_.toStdString()
                                           << " manifest file.");
  return false;
}

bool RosPackage::createCMakeLists()
{
  ROS_FATAL("The RosPackage::createCMakeLists() has not been implemented yet.");
  return false;
}

bool RosPackage::updateManifest()
{
  ROS_FATAL("The RosPackage::updateManifest() has not been implemented yet.");
  return false;
}

QString RosPackage::getPackageUrl()
{
  std::string url;
  rp_.find(name_.toStdString(), url);
  return QString::fromStdString(url);
}

QString RosPackage::getManifestUrl()
{
  QString package_url(getPackageUrl());
  return !package_url.isEmpty() ? package_url + "/package.xml" : "";
}

QString RosPackage::getCMakeListsUrl()
{
  QString package_url(getPackageUrl());
  return !package_url.isEmpty() ? package_url + "/CMakeLists.txt" : "";
}

bool RosPackage::catkinMake() const
{
  if (workspace_url_.isEmpty())
  {
    ROS_ERROR("The workspace url is not defined yet.");
    return false;
  }
  QDir workspace_dir(workspace_url_);
  if (!workspace_dir.exists())
  {
    ROS_ERROR_STREAM("The given workspace url [" << workspace_url_.toStdString()
                                                 << "] does not exist.");
    return false;
  }
  QFileInfo file_info(workspace_url_ + "/src/CMakeLists.txt");
  if (!file_info.exists())
  {
    ROS_ERROR_STREAM("The given url [" << workspace_url_.toStdString()
                                       << "] is not a ROS workspace.");
    return false;
  }
  QString cmd("cd " + workspace_url_ + " && catkin_make");
  system(cmd.toStdString().c_str());
  ROS_INFO_STREAM("Ran the catkin_make command at the given workspace url ["
                  << workspace_url_.toStdString() << "]");
  return true;
}

bool RosPackage::workspaceExists() const
{
  if (workspace_url_.isEmpty())
  {
    return false;
  }
  QFileInfo file_info(workspace_url_ + "/src/CMakeLists.txt");
  return file_info.exists();
}

bool RosPackage::catkinInitWorkspace() const
{
  if (workspaceExists())
  {
    ROS_ERROR_STREAM("The given url [" << workspace_url_.toStdString()
                                       << "] is already a ROS workspace.");
    return false;
  }
  QString workspace_src_url(workspace_url_ + "/src");
  QDir workspace_src_dir(workspace_src_url);
  if (!workspace_src_dir.exists())
  {
    QString cmd("mkdir -p " + workspace_src_url);
    system(cmd.toStdString().c_str());
    if (!workspace_src_dir.exists())
    {
      ROS_ERROR_STREAM(
          "Unable to make directory: " << workspace_src_url.toStdString());
      return false;
    }
    ROS_INFO_STREAM("Made directory: " << workspace_src_url.toStdString());
  }
  QString cmd("cd " + workspace_src_url + " && catkin_init_workspace");
  system(cmd.toStdString().c_str());
  if (!workspaceExists())
  {
    ROS_ERROR_STREAM(
        "Unable to create ROS workspace at: " << workspace_url_.toStdString());
    return false;
  }
  ROS_INFO_STREAM(
      "Created ROS workspace at: " << workspace_src_url.toStdString());
  QDir workspace_dir(workspace_url_);
  if (workspace_dir.exists())
  {
    return false;
  }
  catkinMake();
  QString workspace_devel_url(workspace_url_ + "/devel");
  QString bashrc_url(QDir::homePath() + "/.bashrc");
  QFileInfo bashrc_file_info(bashrc_url);
  if (!bashrc_file_info.exists())
  {
    ROS_ERROR_STREAM("Unable to configure ROS workspace at the ~/bashrc file.");
    return false;
  }
  QFile bashrc_file(bashrc_url);
  if (!bashrc_file.open(QIODevice::ReadWrite | QIODevice::Append |
                        QIODevice::Text))
  {
    ROS_ERROR_STREAM("Unable to open the ~/bashrc file.");
    return false;
  }
  QTextStream in(&bashrc_file);
  QString bashrc_text(in.readAll());
  cmd = "source " + workspace_url_ + "/devel/setup.bash";
  if (bashrc_text.contains(cmd))
  {
    bashrc_file.write(cmd.toStdString().c_str());
  }
  bashrc_file.close();
}

void RosPackage::save(QSettings& settings) const
{
  if (!isValid())
  {
    return;
  }
  settings.beginGroup("package");
  settings.setValue("name", name_);
  settings.setValue("version", version_);
  settings.setValue("description", description_);
  settings.setValue("author", author_);
  // settings.setValue("author", author_email_);
  settings.setValue("maintainer", maintainer_);
  // settings.setValue("maintainer", maintainer_email_);
  settings.setValue("license", license_);
  settings.setValue("buildtool_depend", buildtool_depend_);
  for (size_t i(0); i < build_depends_.count(); i++)
  {
    settings.setValue("build_depend", build_depends_[i]);
  }
  for (size_t i(0); i < run_depends_.count(); i++)
  {
    settings.setValue("run_depend", run_depends_[i]);
  }
  if (!exports_.isEmpty())
  {
    QString exports;
    for (size_t i(0); i < exports_.count(); i++)
    {
      exports += "\n\t" + exports_[i];
    }
    exports += "\n";
    settings.setValue("export", exports);
  }
  settings.endGroup();
}

void RosPackage::load(QSettings& settings)
{
  settings.beginGroup("package");
  setName(settings.value("name").toString());
  std::string url;
  rp_.find(name_.toStdString(), url);
  url = url.substr(0, url.find_last_of('/') - 1);
  setWorkspaceUrl(QString::fromStdString(url));
  ROS_ERROR("[RosPackage] workspace url: %s",
            workspace_url_.toStdString().c_str());
  setVersion(settings.value("version").toString());
  setDescription(settings.value("description").toString());
  setAuthor(settings.value("author").toString());
  setAuthorEmail(settings.value("author").toString());
  ROS_ERROR("[RosPackage] author email: %s",
            author_email_.toStdString().c_str());
  setMaintainer(settings.value("maintainer").toString());
  setMaintainerEmail(settings.value("maintainer").toString());
  ROS_ERROR("[RosPackage] maintainer email: %s",
            maintainer_email_.toStdString().c_str());
  setLicense(settings.value("license").toString());
  setBuildtoolDepend(settings.value("buildtool_depend").toString());
  clearBuildDepends();
  // setBuildtoolDepend(settings.value("build_depend").toString());
  ROS_ERROR("[RosPackage] build_depend: %s",
            settings.value("build_depend").toString().toStdString().c_str());
  clearRunDepends();
  // setBuildtoolDepend(settings.value("run_depend").toString());
  ROS_ERROR("[RosPackage] run_depend: %s",
            settings.value("run_depend").toString().toStdString().c_str());
  clearExports();
  QString exports(settings.value("export").toString());
  ROS_ERROR("[RosPackage] exports: %s", exports.toStdString().c_str());
  settings.endGroup();
}

void RosPackage::reset()
{
  setWorkspaceUrl("");
  setName("");
  setVersion("1.0.0");
  setDescription("");
  setAuthor("Adriano Henrique Rossette Leite");
  setAuthorEmail("adrianohrl@gmail.com");
  setMaintainer("");
  setMaintainerEmail("");
  setLicense("BSD");
  setBuildtoolDepend("catkin");
  clearBuildDepends();
  clearRunDepends();
  clearExports();
}

void RosPackage::write(QDataStream& stream) const
{
  stream << workspace_url_;
  stream << name_;
  stream << version_;
  stream << description_;
  stream << author_;
  stream << author_email_;
  stream << maintainer_;
  stream << maintainer_email_;
  stream << license_;
  stream << buildtool_depend_;
  stream << build_depends_.count();
  for (size_t i(0); i < build_depends_.count(); i++)
  {
    stream << build_depends_[i];
  }
  stream << run_depends_.count();
  for (size_t i(0); i < run_depends_.count(); i++)
  {
    stream << run_depends_[i];
  }
  stream << exports_.count();
  for (size_t i(0); i < exports_.count(); i++)
  {
    stream << exports_[i];
  }
}

void RosPackage::read(QDataStream& stream)
{
  QString workspace_url;
  QString name;
  QString version;
  QString description;
  QString author;
  QString author_email;
  QString maintainer;
  QString maintainer_email;
  QString license;
  QString buildtool_depend;
  quint64 build_depends_count, run_depends_count, exports_count;
  QString build_depend;
  QString run_depend;
  QString exporrt;
  stream >> workspace_url;
  setWorkspaceUrl(workspace_url);
  stream >> name;
  setName(name);
  stream >> version;
  setVersion(version);
  stream >> description;
  setDescription(description);
  stream >> author;
  setAuthor(author);
  stream >> author_email;
  setAuthorEmail(author_email);
  stream >> maintainer;
  setMaintainer(maintainer);
  stream >> maintainer_email;
  setMaintainerEmail(maintainer_email);
  stream >> license;
  setLicense(license);
  stream >> buildtool_depend;
  setBuildtoolDepend(buildtool_depend);
  stream >> build_depends_count;
  for (size_t i(0); i < build_depends_count; i++)
  {
    stream >> build_depend;
    addBuildDepend(build_depend);
  }
  stream >> run_depends_count;
  for (size_t i(0); i < run_depends_count; i++)
  {
    stream >> run_depend;
    addRunDepend(run_depend);
  }
  stream >> exports_count;
  for (size_t i(0); i < exports_count; i++)
  {
    stream >> exporrt;
    addExport(exporrt);
  }
}

RosPackage& RosPackage::operator=(const RosPackage& config)
{
  setWorkspaceUrl(config.workspace_url_);
  setName(config.name_);
  setVersion(config.version_);
  setDescription(config.description_);
  setAuthor(config.author_);
  setAuthorEmail(config.author_email_);
  setMaintainer(config.maintainer_);
  setMaintainerEmail(config.maintainer_email_);
  setLicense(config.license_);
  setBuildtoolDepend(config.buildtool_depend_);
  setBuildDepends(config.build_depends_);
  setRunDepends(config.run_depends_);
  setExports(config.exports_);
}

RosMetapackage::RosMetapackage(QObject* parent) : RosPackage(parent) {}

RosMetapackage::~RosMetapackage() {}

void RosMetapackage::reset()
{
  RosPackage::reset();
  addExport("<metapackage/>");
}

bool RosMetapackage::createCMakeLists()
{
  if (!isValidPackageName())
  {
    ROS_ERROR_STREAM("The given package name [" << name_.toStdString()
                                                << "] is not valid.");
    return false;
  }
  QString package_url(getPackageUrl());
  if (package_url.isEmpty())
  {
    ROS_ERROR_STREAM("The given package [" << name_.toStdString()
                                           << "] doest not exist.");
    return false;
  }
  QFile file(package_url + "/CMakeLists.txt");
  if (file.exists())
  {
    ROS_ERROR_STREAM("The given package ["
                     << name_.toStdString()
                     << "] already has its CMakeLists.txt file.");
    return false;
  }
  QString cmake_lists_text;
  cmake_lists_text += "cmake_minimum_required(VERSION 2.8.3)\n";
  cmake_lists_text += "project(" + name_ + ")\n";
  cmake_lists_text += "find_package(catkin REQUIRED)";
  cmake_lists_text += "catkin_metapackage()";
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    ROS_ERROR_STREAM("Unable to open the " << package_url.toStdString()
                                           << "/CMakeLists.txt file.");
    return false;
  }
  file.write(cmake_lists_text.toStdString().c_str());
  file.close();
  return true;
}
}
