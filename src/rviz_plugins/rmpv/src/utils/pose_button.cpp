/**
* *********************************************************
*
* @file: pose_button.cpp
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
#include "utils/pose_button.h"

namespace rmpv
{
/*
  * @brief Construct a new PoseButton object
  * @param parent  the parent widget
 */
PoseButton::PoseButton(QWidget* parent)
 : QPushButton(parent)
{
  setFixedSize(25, 25);
  setIcon(QIcon(":/icons/pose.png"));
  setToolTip("Set pose using mouse");
}
}  // namespace rmpv