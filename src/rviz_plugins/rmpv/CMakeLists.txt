cmake_minimum_required(VERSION 3.5)

project(rmpv LANGUAGES CXX)

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
        src/rmpv/rmpv.cpp
        src/path_visualizer/panel_path_visualizer.cpp
        src/path_visualizer/core_path_visualizer.cpp
        src/path_visualizer/path_list.cpp
        src/path_visualizer/path_info.cpp
        src/curve_visualizer/panel_curve_visualizer.cpp
        src/curve_visualizer/core_curve_visualizer.cpp
        src/curve_visualizer/curve_list.cpp
        src/curve_visualizer/curve_info.cpp
        src/utils/pose_2d.cpp
        src/utils/select_delegate.cpp
        src/utils/color_editor.cpp
#        src/utils/pose_button.cpp
#        src/utils/start_pose_button.cpp
#        src/utils/goal_pose_button.cpp
        ${MOC_FILES}
        )

set(HEADER_FILES
        include/rmpv/rmpv.h
        include/rmpv/ui_rmpv.h
        include/path_visualizer/panel_path_visualizer.h
        include/path_visualizer/core_path_visualizer.h
        include/path_visualizer/path_list.h
        include/path_visualizer/path_info.h
        include/curve_visualizer/panel_curve_visualizer.h
        include/curve_visualizer/core_curve_visualizer.h
        include/curve_visualizer/curve_list.h
        include/curve_visualizer/curve_info.h
        include/utils/point_2d.h
        include/utils/pose_2d.h
        include/utils/select_delegate.h
        include/utils/color_editor.h
#        include/utils/pose_button.h
#        include/utils/start_pose_button.h
#        include/utils/goal_pose_button.h
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
