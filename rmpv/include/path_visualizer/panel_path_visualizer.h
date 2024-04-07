/**
 * *********************************************************
 *
 * @file: panel_path_visualizer.h
 * @brief: Contains panel of path visualizer class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2024-4-6
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong, Wu Maojia. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef PANEL_PATH_VISUALIZER_H
#define PANEL_PATH_VISUALIZER_H

#include <QWidget>
#include <QRegExpValidator>
#include <QStandardItemModel>
#include <QCheckBox>
#include <QColorDialog>
#include <QRegularExpression>
#include <QFileDialog>

#include <rviz/panel.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/color_editor.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "wrapper_planner/CallPlan.h"
#include "utils/select_delegate.h"
#include "utils/color_editor.h"
#include <QToolButton>
#include "path_visualizer/core_path_visualizer.h"
#include "rmpv/ui_rmpv.h"

namespace rmpv
{
class PanelPathVisualizer : public QObject
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new PanelPathVisualizer object
   * @param parent: parent widget rmpv
   * @param ui: ui object
   */
  PanelPathVisualizer(rviz::Panel* parent_ = nullptr, Ui::RMPV* ui = nullptr);

  /**
   * @brief Destroy the PanelPathVisualizer object
   */
  ~PanelPathVisualizer();

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
   *  @brief if color changed signal from colorEditor is received, call this slot function
   *  @param index  row index of path
   *  @param color  color of path
   */
  void _onColorChanged(const int &index, const QColor &color);

  /**
   *  @brief if select changed signal from selectDelegate is received, call this slot function
   *  @param index  row index of path
   *  @param checked  if path is selected
   */
  void _onSelectChanged(const int &index, const bool &checked);

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
  rviz::Panel* parent_;       // parent panel rmpv
  Ui::RMPV* ui_;              // ui object
  CorePathVisualizer* core_;  // core object

  QStandardItemModel* table_model_; // model of table "Path List"
  QStringList table_header_;        // header of table "Path List"

  selectDelegate selectDelegate_;  // delegate for select column
};
}  // namespace rmpv

#endif  // PANEL_PATH_VISUALIZER_H
