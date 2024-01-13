/***********************************************************
*
* @file: panel_curve_visualizer.h
* @breif: Contains panel of curve visualizer class
* @author: Wu Maojia, Yang Haodong
* @update: 2024-1-12
* @version: 1.0
*
* Copyright (c) 2024， Yang Haodong, Wu Maojia
* All rights reserved.
* --------------------------------------------------------
*
**********************************************************/
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

private:
  rviz::Panel* parent_;       // parent panel rmpv
  Ui::RMPV* ui_;              // ui object
  CoreCurveVisualizer* core_;  // core object
};
}  // namespace rmpv

#endif  // PANEL_CURVE_VISUALIZER_H
