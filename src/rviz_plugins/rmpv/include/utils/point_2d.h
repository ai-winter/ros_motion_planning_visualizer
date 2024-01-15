/**
 * *********************************************************
 *
 * @file: point_2d.h
 * @brief: Contains Point2D struct
 * @author: Wu Maojia, Yang Haodong
 * @date: 2024-01-12
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong, Wu Maojia. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef POINT_2D_H
#define POINT_2D_H

#include <QObject>

namespace rmpv
{
class Point2D
{
public:
  /**
   * @brief Construct a new Point2D object
   * @param x  the x coordinate of point
   * @param y  the y coordinate of point
   */
  Point2D(double x = 0.0, double y = 0.0): x(x), y(y) {};

  /**
   * @brief Destroy the Point2D object
   */
  ~Point2D(){};

public:
  double x, y;  // the x and y coordinate of point
};
}  // namespace rmpv
Q_DECLARE_METATYPE(rmpv::Point2D)
#endif  // POINT_2D_H
