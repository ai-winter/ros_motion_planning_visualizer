/**
 * *********************************************************
 *
 * @file: curve_info.cpp
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
#include "curve_visualizer/curve_info.h"

namespace rmpv
{
/**
 * @brief Construct a new CurveInfo object
 * @param tp  the type of curve
 * @param pss  the poses used to plan the curve
 * @param crv  the curve points
 * @param c  the color of curve
 * @param slt  whether the curve is selected
 */
CurveInfo::CurveInfo(QString tp, QList<Pose2D> pss, QList<Point2D> crv, QColor c, bool slt)
{
  type_ = tp;
  poses_ = pss;
  curve_ = crv;
  color_ = c;
  select_ = slt;

  length_ = _calcCurveLength();
  turning_angle_ = _calcTurningAngle();
}

/**
 * @brief Destroy the CurveInfo object
 */
CurveInfo::~CurveInfo()
{
}

/*
 * @brief get some data from curve info
 * @param variant  specify the data to get
 * @return the desired data
 */
QVariant CurveInfo::getData(const int& variant) const
{
  switch (variant)
  {
    case curveType:
      return type_;
    case curveLength:
      return length_;
    case curveColor:
      return color_;
    case turningAngle:
      return turning_angle_;
    case selectStatus:
      return select_;
    default:
      return QVariant();
  }
}

/*
 * @brief get curve poses data from curve info
 * @return the list of curve poses
 */
QList<Pose2D> CurveInfo::getCurvePoses() const
{
  return poses_;
}

/*
 * @brief get curve points data from curve info
 * @return the list of curve points
 */
QList<Point2D> CurveInfo::getCurvePoints() const
{
  return curve_;
}

/*
 * @brief set some data of curve info
 * @param variant  specify the data to set
 * @return true if set successfully
 */
bool CurveInfo::setData(const int& variant, const QVariant& value)
{
  switch (variant)
  {
    case curveColor:
      color_ = value.value<QColor>();
      break;
    case selectStatus:
      select_ = value.toBool();
      break;
    default:
      return false;
  }
  return true;
}

/**
 * @brief return the size of curve info (number of poses)
 * @return the size of curve info
 */
int CurveInfo::size() const
{
  return poses_.size();
}

/*
 * @brief Calculate curve length
 * @return curve length
 */
double CurveInfo::_calcCurveLength()
{
  double length = 0.0;
  for (int i = 0; i < curve_.size() - 1; i++)
    length += std::sqrt(std::pow(curve_[i + 1].x - curve_[i].x, 2) + std::pow(curve_[i + 1].y - curve_[i].y, 2));

  return length;
}

/*
 * @brief Calculate curve turning angle
 * @return curve turning angle
 */
double CurveInfo::_calcTurningAngle()
{
  double angle = 0.0;
  for (int i = 0; i < curve_.size() - 2; i++)
  {
    double x1 = curve_[i + 1].x - curve_[i].x;
    double y1 = curve_[i + 1].y - curve_[i].y;
    double x2 = curve_[i + 2].x - curve_[i + 1].x;
    double y2 = curve_[i + 2].y - curve_[i + 1].y;
    angle += std::acos(
        std::max(-1.0, std::min(1.0,
                              (x1 * x2 + y1 * y2) / (std::sqrt(x1 * x1 + y1 * y1) * std::sqrt(x2 * x2 + y2 * y2))
                              )));
  }
  // take theta angle into account
  angle += std::abs(poses_[0].theta - poses_[poses_.size() - 1].theta);
  return angle;
}
}  // namespace rmpv
