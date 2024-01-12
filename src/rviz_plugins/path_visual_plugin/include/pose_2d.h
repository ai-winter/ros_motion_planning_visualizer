/***********************************************************
 *
 * @file: point_2d.h
 * @breif: Contains Point2D struct
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-11-2
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef POINT_2D_H
#define POINT_2D_H

namespace path_visual_plugin
{
struct Point2D
{
public:
  /**
   * @brief Construct a new Point2D object
   * @param x  the x coordinate of point
   * @param y  the y coordinate of point
   */
  Point2D(double x = 0.0, double y = 0.0): x(x), y(y) {};

public:
  double x, y;  // the x and y coordinate of point
};
}  // namespace path_visual_plugin
Q_DECLARE_METATYPE(path_visual_plugin::Point2D)
#endif  // POINT_2D_H
