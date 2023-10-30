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
#include <QFileDialog>
#include <rviz/panel.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "include/table_delegate.h"

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
   *  @brief if clicked signal from pushButton is received, call this slot function
   */
  void _onClicked();

  /**
   *  @brief if editing finished signal from lineEdit is received, call this slot function
   */
  void _onEditingFinished();

  /**
   *  @brief if state changed signal is from checkBox received, call this slot function
   */
  void _onCheckBoxStateChanged(const QModelIndex &index, const bool &checked);
  void _onStateChanged(int state);

  /**
   *  @brief if value changed signal from core is received, call this slot function
   */
  void _onValueChanged();

protected:
  /**
   *  @brief update the table_model_ to update the table view of Path List
   */
  void _updateTableView();

private:
  Ui::PathVisualPlugin* ui_;   // ui object
  CorePathVisualPlugin* core_; // core object

  QStandardItemModel* table_model_; // model of table "Path List"
  QStringList table_header_;        // header of table "Path List"

  CheckBoxListSelectDelegate checkBoxListSelectDelegate_;  // delegate for checkBoxListSelect
};

}  // namespace path_visual_plugin
#endif  // PATH_VISUAL_PLUGIN_H
