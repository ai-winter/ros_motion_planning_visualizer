/**
* *********************************************************
*
* @file: start_pose_button.h
* @brief: Contains start pose pushButton class
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
#ifndef START_POSE_BUTTON_H
#define START_POSE_BUTTON_H

#include "utils/pose_button.h"

namespace rmpv
{
class StartPoseButton : public PoseButton
{
 Q_OBJECT

public:
  /*
   * @brief Construct a new StartPoseButton object
   * @param parent  the parent widget
   */
  StartPoseButton(QWidget* parent = nullptr);

  ~StartPoseButton();

protected:
  void onPoseSet(double x, double y, double theta) override;
};
} // namespace rmpv
#endif  // START_POSE_BUTTON_H
