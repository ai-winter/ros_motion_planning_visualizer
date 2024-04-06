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
#ifndef RMPV_H
#define RMPV_H

#include <rviz/panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>

#include "rmpv/ui_rmpv.h"
#include "path_visualizer/panel_path_visualizer.h"
#include "curve_visualizer/panel_curve_visualizer.h"

namespace rmpv
{
class RMPV : public rviz::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new RMPV object
   * @param parent
   */
  RMPV(QWidget* parent = nullptr, rviz::VisualizationManager* manager = nullptr);

  /**
   * @brief Destroy the RMPV object
   */
  ~RMPV();

private:
  Ui::RMPV* ui_;   // ui object
  PanelPathVisualizer* panel_path_; // panel of path visualizer
  PanelCurveVisualizer* panel_curve_; // panel of curve visualizer
  rviz::VisualizationManager* manager_; // Rviz VisualizationManager object
};

}  // namespace rmpv
#endif  // RMPV_H
