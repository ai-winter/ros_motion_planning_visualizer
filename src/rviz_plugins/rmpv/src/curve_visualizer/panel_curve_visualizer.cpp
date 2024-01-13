/***********************************************************
*
* @file: panel_curve_visualizer.h
* @breif: Contains panel of curve visualizer class
* @author: Wu Maojia, Yang Haodong
* @update: 2024-1-13
* @version: 1.0
*
* Copyright (c) 2024， Yang Haodong, Wu Maojia
* All rights reserved.
* --------------------------------------------------------
*
**********************************************************/
#include "curve_visualizer/panel_curve_visualizer.h"

namespace rmpv
{
/**
 * @brief Construct a new PanelCurveVisualizer object
 */
PanelCurveVisualizer::PanelCurveVisualizer(rviz::Panel* parent, Ui::RMPV* ui)
  : parent_(parent), ui_(ui), core_(new CoreCurveVisualizer)
{
  core_->setupROS();
  setupUi();
}

/**
* @brief Destroy the PanelCurveVisualizer object
 */
PanelCurveVisualizer::~PanelCurveVisualizer()
{
  if (ui_)
    delete ui_;
  if (core_)
    delete core_;
}

/**
* @brief User interface parameters initialization
 */
void PanelCurveVisualizer::setupUi()
{
}

}  // namespace rmpv