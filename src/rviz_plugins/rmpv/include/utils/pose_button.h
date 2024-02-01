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
#include <QPushButton>

namespace rmpv
{
class PoseButton : public QPushButton
{
 Q_OBJECT

public:
 /*
  * @brief Construct a new PoseButton object
  * @param parent  the parent widget
  */
 PoseButton(QWidget* parent = nullptr);
};
} // namespace rmpv
#endif  // POSE_BUTTON_H
