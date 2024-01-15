/**
 * *********************************************************
 *
 * @file: panel_curve_visualizer.h
 * @brief: Contains panel of curve visualizer class
 * @author: Wu Maojia
 * @date: 2024-01-13
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong, Wu Maojia. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef PANEL_CURVE_VISUALIZER_H
#define PANEL_CURVE_VISUALIZER_H

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
#include "curve_visualizer/core_curve_visualizer.h"
#include "rmpv/ui_rmpv.h"

namespace rmpv
{
class PanelCurveVisualizer : public QObject
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new PanelCurveVisualizer object
   * @param parent: parent widget rmpv
   * @param ui: ui object
   */
  PanelCurveVisualizer(rviz::Panel* parent_ = nullptr, Ui::RMPV* ui = nullptr);

  /**
   * @brief Destroy the PanelCurveVisualizer object
   */
  ~PanelCurveVisualizer();

  /**
   * @brief User interface parameters initialization
   */
  void setupUi();

protected Q_SLOTS:
  /**
   *  @brief if clicked signal from pushButton is received, call this slot function
   */
  void _onClicked();

protected:
  /**
   *  @brief update the curves_model_ to update the table view of Curve List
   */
  void _updateTableViewCurves();

  /**
   *  @brief update the poses_model_ to update the table view of Pose List
   */
  void _updateTableViewPoses();

private:
  rviz::Panel* parent_;       // parent panel rmpv
  Ui::RMPV* ui_;              // ui object
  CoreCurveVisualizer* core_;  // core object

  QStandardItemModel* curves_model_; // model of table "Curve List"
  QStringList curves_header_;        // header of table "Curve List"

  QStandardItemModel* poses_model_;  // model of table "Poses List"
  QStringList poses_header_;         // header of table "Poses List"
};
}  // namespace rmpv

#endif  // PANEL_CURVE_VISUALIZER_H
