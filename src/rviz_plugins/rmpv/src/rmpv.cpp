/***********************************************************
*
* @file: rmpv.cpp
* @breif: Contains ROS Motion Planning Visualizer (RMPV) Rviz plugin class
* @author: Wu Maojia, Yang Haodong
* @update: 2024-1-12
* @version: 1.0
*
* Copyright (c) 2023， Yang Haodong, Wu Maojia
* All rights reserved.
* --------------------------------------------------------
*
**********************************************************/
#include "include/rmpv.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rmpv::RMPV, rviz::Panel)

namespace rmpv
{
/**
* @brief Construct a new RMPV object
*/
RMPV::RMPV(QWidget* parent)
 : rviz::Panel(parent), ui_(new Ui::RMPV)
{
  ui_->setupUi(this);
  panel_path_ = new PanelPathVisualizer(this, ui_);
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
}
}  // namespace rmpv