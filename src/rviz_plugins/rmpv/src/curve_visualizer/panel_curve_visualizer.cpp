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
#include "curve_visualizer/panel_curve_visualizer.h"

namespace rmpv
{
/**
 * @brief Construct a new PanelCurveVisualizer object
 * @param parent: parent widget rmpv
 * @param ui: ui object
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
  if (curves_model_)
    delete curves_model_;
  if (poses_model_)
    delete poses_model_;
}

/**
 * @brief User interface parameters initialization
 */
void PanelCurveVisualizer::setupUi()
{
  curves_model_ = new QStandardItemModel(parent_);
  curves_header_ = QStringList({ "Select", "Type", "Length", "Turning Angle", "Color", "Remove" });
  ui_->tableView_curve_curves->setModel(curves_model_);
  _updateTableViewCurves();

  poses_model_ = new QStandardItemModel(parent_);
  poses_header_ = QStringList({ "x", "y", "\316\270", "Remove" });
  ui_->tableView_curve_poses->setModel(poses_model_);
  _updateTableViewPoses();
}

/**
 *  @brief update the curves_model_ to update the table view of Curve List
 */
void PanelCurveVisualizer::_updateTableViewCurves()
{
  // initialize curve list table model
  curves_model_->clear();
  curves_model_->setHorizontalHeaderLabels(curves_header_);
  curves_model_->setHeaderData(0, Qt::Horizontal, "Only the selected curves will be displayed or saved.",
                               Qt::ToolTipRole);
  curves_model_->setHeaderData(1, Qt::Horizontal, "Type of the generated curve.", Qt::ToolTipRole);
  curves_model_->setHeaderData(2, Qt::Horizontal, "Total length of the curve", Qt::ToolTipRole);
  curves_model_->setHeaderData(3, Qt::Horizontal, "Total turning angle of the curve, in radians", Qt::ToolTipRole);
  curves_model_->setHeaderData(4, Qt::Horizontal, "Color of the curve displayed in rviz", Qt::ToolTipRole);
  curves_model_->setHeaderData(5, Qt::Horizontal, "Remove the curve from curve list", Qt::ToolTipRole);
  //  curves_model_->setRowCount(core_->curve_list_->size());
}

/**
 *  @brief update the poses_model_ to update the table view of Pose List
 */
void PanelCurveVisualizer::_updateTableViewPoses()
{
  // initialize pose list table model
  poses_model_->clear();
  poses_model_->setHorizontalHeaderLabels(poses_header_);
  poses_model_->setHeaderData(0, Qt::Horizontal, "X-coordinate of the pose", Qt::ToolTipRole);
  poses_model_->setHeaderData(1, Qt::Horizontal, "Y-coordinate of the pose", Qt::ToolTipRole);
  poses_model_->setHeaderData(2, Qt::Horizontal, "\316\270 angle of the pose, in radians", Qt::ToolTipRole);
  poses_model_->setHeaderData(3, Qt::Horizontal, "Remove the path from path list", Qt::ToolTipRole);
  //  poses_model_->setRowCount(core_->pose_list_->size());
}

}  // namespace rmpv