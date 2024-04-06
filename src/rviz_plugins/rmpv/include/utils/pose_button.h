/**
* *********************************************************
*
* @file: pose_button.h
* @brief: Contains pose pushButton class
* @author: Wu Maojia
* @date: 2024-2-1
* @version: 1.0
*
* Copyright (c) 2024, Yang Haodong, Wu Maojia.
* All rights reserved.
*
* --------------------------------------------------------
*
* ********************************************************
*/
#ifndef POSE_BUTTON_H
#define POSE_BUTTON_H

#include <QtWidgets>
#include <QToolButton>

#include <rviz/tool_manager.h>

namespace rmpv
{
class PoseButton : public QToolButton
{
 Q_OBJECT

public:
  /*
   * @brief Construct a new ToolButton object
   * @param parent  the parent widget
   */
  PoseButton(QWidget* parent = nullptr);

  ~PoseButton();

protected:
  void onClicked();

  virtual void onPoseSet(double x, double y, double theta) = 0;
};
} // namespace rmpv
#endif  // POSE_BUTTON_H
