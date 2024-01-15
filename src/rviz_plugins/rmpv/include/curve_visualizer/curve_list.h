/**
 * *********************************************************
 *
 * @file: curve_list.h
 * @brief: Contains CurveList class
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
#ifndef CURVE_LIST_H
#define CURVE_LIST_H

#define MAX_CURVE_NUM 10000  // the max number of curves in curve list

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <ros/ros.h>

#include "curve_visualizer/curve_info.h"

namespace rmpv
{
class CurveList
{
public:
  /**
   * @brief Construct a new CurveList object
   */
  CurveList();

  /**
   * @brief Destroy the CurveList object
   */
  ~CurveList();

  /**
   * @brief append a new curve to the curve list
   * @param curve  the new curve to append
   * @return true if append successfully
   */
  bool append(const CurveInfo& curve);

  /**
   * @brief remove the curve with some index from curve list
   * @param index the index of curve to remove
   * @return true if remove successfully
   */
  bool remove(const int& index);

  /**
   * @brief set the color of curve with some index
   * @param index the index of curve to set color
   * @param color the color to set
   * @return true if set successfully
   */
  bool setColor(const int& index, const QColor& color);

  /**
   * @brief set the select status of curve with some index
   * @param index the index of curve to set select status
   * @param select  whether to select and visualize the curve or not
   * @return true if set successfully
   */
  bool setSelect(const int& index, const bool& select);

  /**
   * @brief get the pointer of curve info list
   * @return the pointer of curve info list
   */
  const QList<CurveInfo>* getListPtr() const;

  /**
   * @brief save the selected curves to a local JSON file
   * @param file_name  the file name to save
   * @return true if save successfully
   */
  bool save(QString file_name) const;

  /**
   * @brief load the curves from a local JSON file
   * @param file_name  the file name to load
   * @return true if load successfully
   */
  bool load(QString file_name);

  /**
   * @brief return the size of curve list
   * @return the size of curve list
   */
  int size() const;

private:
  QList<CurveInfo> curves_; // the curve list
};
}  // namespace rmpv
#endif  // CURVE_LIST_H
