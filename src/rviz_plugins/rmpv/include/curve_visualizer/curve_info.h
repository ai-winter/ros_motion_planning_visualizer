/**
 * *********************************************************
 *
 * @file: curve_info.h
 * @brief: Contains CurveInfo class
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
#ifndef CURVE_INFO_H
#define CURVE_INFO_H

#include <cmath>

#include <QColor>
#include <QString>
#include <QList>
#include <QVariant>

#include "utils/point_2d.h"
#include "utils/pose_2d.h"

namespace rmpv
{
class CurveInfo
{
public:
  enum { curveType, curveLength, curveColor, turningAngle, selectStatus };

public:
  /**
   * @brief Construct a new CurveInfo object with parameters
   * @param tp  the type of curve
   * @param pss  the poses used to plan the curve
   * @param crv  the curve points
   * @param c  the color of curve
   * @param slt  whether the curve is selected
   */
  CurveInfo(QString tp = "None", QList<Pose2D> pss = QList<Pose2D>(), QList<Point2D> crv = QList<Point2D>(),
      QColor c = Qt::darkBlue, bool slt = true);

  /**
   * @brief Destroy the CurveInfo object
   */
  ~CurveInfo();

  /*
   * @brief get some data from curve info
   * @param variant  specify the data to get
   * @return the desired data
   */
  QVariant getData(const int& variant) const;

  /*
   * @brief get curve poses data from curve info
   * @return the list of curve poses
   */
  QList<Pose2D> getCurvePoses() const;

  /*
   * @brief get curve points data from curve info
   * @return the list of curve points
   */
  QList<Point2D> getCurvePoints() const;

  /*
   * @brief set some data of curve info
   * @param variant  specify the data to set
   * @return true if set successfully
   */
  bool setData(const int& variant, const QVariant& value);

  /**
   * @brief return the size of curve info (number of poses)
   * @return the size of curve info
   */
  int size() const;

protected:

  /*
   * @brief Calculate curve length
   * @return curve length
   */
  double _calcCurveLength();

  /*
   * @brief Calculate curve turning angle
   * @return curve turning angle
   */
  double _calcTurningAngle();

private:
  // the type of curve
  QString type_;

  // poses used to plan the curve
  QList<Pose2D> poses_;

  // curve planned
  QList<Point2D> curve_;

  // curve length and total turning angle
  double length_, turning_angle_;

  // the color of visualized curve
  QColor color_;

  // select and visualize the curve
  bool select_;
};
}  // namespace rmpv
#endif  // CURVE_INFO_H
