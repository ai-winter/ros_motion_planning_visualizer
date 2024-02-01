/**
* *********************************************************
*
* @file: goal_pose_button.h
* @brief: Contains goal pose pushButton class
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
#ifndef GOAL_POSE_BUTTON_H
#define GOAL_POSE_BUTTON_H

#include "utils/pose_button.h"

namespace rmpv
{
class GoalPoseButton : public PoseButton
{
 Q_OBJECT

public:
 /*
  * @brief Construct a new GoalPoseButton object
  * @param parent  the parent widget
  */
 GoalPoseButton(QWidget* parent = nullptr);
};
} // namespace rmpv
#endif  // GOAL_POSE_BUTTON_H
