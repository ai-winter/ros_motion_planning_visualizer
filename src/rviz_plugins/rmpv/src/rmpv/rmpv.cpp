/**
 * *********************************************************
 *
 * @file: rmpv.h
 * @brief: Contains ROS Motion Planning Visualizer (RMPV) Rviz plugin class
 * @author: Wu Maojia, Yang Haodong
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
#include "rmpv/rmpv.h"
#include <rviz/geometry.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rmpv::RMPV, rviz::Panel)

namespace rmpv
{
/**
 * @brief Construct a new RMPV object
 * @param parent
 */
RMPV::RMPV(QWidget* parent, rviz::VisualizationManager* manager) : rviz::Panel(parent), ui_(new Ui::RMPV), manager_(manager)
{
  ui_->setupUi(this);
  panel_path_ = new PanelPathVisualizer(this, ui_);
  panel_curve_ = new PanelCurveVisualizer(this, ui_);
  if (parent)
  {
    ROS_WARN("parent is not null");
  }else{
    ROS_WARN("parent is null");
  }
  if (manager_)
  {
    ROS_WARN("manager_ is not null");
//    panel_path_->setVisualizationManager(manager_);
//    panel_curve_->setVisualizationManager(manager_);
  }else{
    ROS_WARN("manager_ is null");
  }
}

/**
 * @brief Destroy the RMPV object
 */
RMPV::~RMPV()
{
  if (ui_)
    delete ui_;
  if (panel_path_)
    delete panel_path_;
  if (panel_curve_)
    delete panel_curve_;
  if (manager_)
    delete manager_;
}
}
}  // namespace rmpv