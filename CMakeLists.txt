cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(rqt_mrta)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  variant_topic_tools
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ${rqt_mrta_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rqt_gui
    rqt_gui_cpp
    variant_topic_tools
)

set (rqt_mrta_SRCS
  src/mrta/architecture.cpp
  src/mrta/architecture_config.cpp
  src/mrta/taxonomy.cpp
  src/rqt_mrta/abstract_topic_monitor_config.cpp
  src/rqt_mrta/abstract_topic_monitor_widget.cpp
  src/rqt_mrta/allocated_tasks_config.cpp
  src/rqt_mrta/architecture_selection_config.cpp
  src/rqt_mrta/architecture_selection_widget.cpp
  src/rqt_mrta/busy_robots_config.cpp
  src/rqt_mrta/idle_robots_config.cpp
  src/rqt_mrta/incoming_tasks_config.cpp
  src/rqt_mrta/mrta_plugin.cpp
  src/rqt_mrta/mrta_widget.cpp
  src/rqt_mrta/topic_config.cpp
  src/utilities/abstract_config.cpp
  src/utilities/xml_settings.cpp
)

set(rqt_mrta_HDRS
  include/mrta/architecture.h
  include/mrta/architecture_config.h
  include/mrta/taxonomy.h
  include/rqt_mrta/abstract_topic_monitor_config.h
  include/rqt_mrta/abstract_topic_monitor_widget.h
  include/rqt_mrta/allocated_tasks_config.h
  include/rqt_mrta/architecture_selection_config.h
  include/rqt_mrta/architecture_selection_widget.h
  include/rqt_mrta/busy_robots_config.h
  include/rqt_mrta/idle_robots_config.h
  include/rqt_mrta/incoming_tasks_config.h
  include/rqt_mrta/mrta_plugin.h
  include/rqt_mrta/mrta_widget.h
  include/rqt_mrta/topic_config.h
  include/utilities/abstract_config.h
  include/utilities/xml_settings.h
)

set(rqt_mrta_UIS
  src/rqt_mrta/abstract_topic_monitor_widget.ui
  src/rqt_mrta/architecture_selection_widget.ui
  src/rqt_mrta/mrta_widget.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_mrta_MOCS ${rqt_mrta_HDRS})
  qt5_wrap_ui(rqt_mrta_UIS_H ${rqt_mrta_UIS})
else()
  qt4_wrap_cpp(rqt_mrta_MOCS ${rqt_mrta_HDRS})
  qt4_wrap_ui(rqt_mrta_UIS_H ${rqt_mrta_UIS})
endif()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${mrta_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${rqt_mrta_SRCS}
  ${rqt_mrta_MOCS}
  ${rqt_mrta_UIS_H}
)

add_dependencies(${PROJECT_NAME}
  ${mrta_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${mrta_LIBRARIES}
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/rqt_mrta
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
