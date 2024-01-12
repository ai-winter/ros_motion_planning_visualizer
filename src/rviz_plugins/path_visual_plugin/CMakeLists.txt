cmake_minimum_required(VERSION 3.5)

project(path_visual LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
        rviz
        wrapper_planner
        pluginlib
        )

catkin_package(
        CATKIN_DEPENDS message_runtime wrapper_planner
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(
        ${catkin_LIBRARY_DIRS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# if you want to modify the .ui file to change the ui_xxx_plugin.h,
# please run the following command in the root directory of the plugin:
#       uic -o include/ui_path_visual_plugin.h  ui/path_visual_plugin.ui
set(CMAKE_AUTOUIC OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(Qt5 COMPONENTS Widgets REQUIRED)

if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
        src/path_visual_plugin.cpp
        src/core_path_visual_plugin.cpp
        src/path_list.cpp
        src/path_info.cpp
        src/pose_2d.cpp
        src/select_delegate.cpp
        src/color_editor.cpp
        ${MOC_FILES}
        )

set(HEADER_FILES
        include/path_visual_plugin.h
        include/core_path_visual_plugin.h
        include/path_list.h
        include/path_info.h
        include/point_2d.h
        include/pose_2d.h
        include/select_delegate.h
        include/color_editor.h
        include/ui_path_visual_plugin.h
        )

set(RESOURCE_FILES
        resources/resources.qrc
        )

add_library(${PROJECT_NAME}
        ${SOURCE_FILES}
        ${HEADER_FILES}
        ${RESOURCE_FILES}
        )

target_link_libraries(${PROJECT_NAME}
        ${QT_LIBRARIES}
        ${catkin_LIBRARIES}
        )

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)
