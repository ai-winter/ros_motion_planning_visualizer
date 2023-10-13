/***********************************************************
 *
 * @file: path_visual_plugin.h
 * @breif: Contains path visualization Rviz plugin class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-10-14
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PATH_VISUAL_PLUGIN_H
#define PATH_VISUAL_PLUGIN_H

#include <QWidget>
#include <QRegExpValidator>
#include <QStandardItemModel>
#include <QCheckBox>
#include <QColorDialog>
#include <QRegularExpression>
#include <rviz/panel.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

QT_BEGIN_NAMESPACE
namespace Ui
{
class PathVisualPlugin;
}
QT_END_NAMESPACE

namespace path_visual_plugin
{
class CorePathVisualPlugin;

class PathVisualPlugin : public rviz::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new Path Visualization Plugin object
   */
  PathVisualPlugin(QWidget* parent = nullptr);

  /**
   * @brief Destroy the Path Visualization Plugin object
   */
  ~PathVisualPlugin();

  /**
   * @brief User interface parameters initialization
   */
  void setupUi();

protected Q_SLOTS:
  /**
   *  @brief if clicked signal is received, call this slot function
   */
  void _onClicked();

  /**
   *  @brief if editing finished signal is received, call this slot function
   */
  void _onEditingFinished();

  /**
   *  @brief if value changed signal from core is received, call this slot function
   */
  void _onValueChanged();

  /**
   *  @brief add a path row in table view
   */
  void _addPathRow();

  /**
   *  @brief remove a path with some index row in table view
   *  @param index  the index of the removed path
   */
  void _removePathRow(const int& index);

private:
  Ui::PathVisualPlugin* ui;   // ui object
  CorePathVisualPlugin* core; // core object

  QStandardItemModel* table_model_; // model of table "Path List"
  QStringList table_header_;        // header of table "Path List"
};

}  // namespace path_visual_plugin
#endif  // PATH_VISUAL_PLUGIN_H
