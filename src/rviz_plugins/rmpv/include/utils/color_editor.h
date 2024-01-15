/**
 * *********************************************************
 *
 * @file: color_editor.h
 * @brief: Contains color editor class
 * @author: Wu Maojia
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
#ifndef COLOR_EDITOR_H
#define COLOR_EDITOR_H

#include <QtWidgets>
#include <QStyledItemDelegate>

#include <QMetaObject>
#include <QMetaProperty>

#include <QPainter>
#include <QColor>
#include <QColorDialog>

#include <rviz/properties/parse_color.h>
#include <rviz/properties/line_edit_with_button.h>

namespace rmpv
{
class ColorEditor : public rviz::LineEditWithButton
{
  Q_OBJECT

public:
  /*
   * @brief Construct a new Color Editor object
   * @param index  the row index of color editor in table model
   * @param color  the color of path
   * @param parent  the parent widget
   */
  ColorEditor(int index, const QColor& color, QWidget* parent = nullptr);

  /*
   * @brief Slot function to paint just the color box. Paints it in the left end of rect, size rect.height() by rect.height().
   * @param painter  the painter
   * @param rect  the QT rectangle
   * @param color  the color
   */
  static void paintColorBox(QPainter* painter, const QRect& rect, const QColor& color);

Q_SIGNALS:
  /*
   * @brief Signal emitted when color is changed
   * @param index  the row index of color editor in table model
   * @param color  the new color
   */
  void colorChanged(const int &index, const QColor& color);

public Q_SLOTS:
  /*
   * @brief Set the color
   * @param color  the new color to set
   */
  void setColor(const QColor& color);

  /*
   * @brief Parse the text and update the color
   */
  void parseText();

protected:
  /*
   * @brief Slot function to call parent version then paint color swatch
   * @param event  the paint event
   */
  void paintEvent(QPaintEvent* event) override;

  /*
   * @brief Resize the color editor
   * @param event  the resize event
   */
  void resizeEvent(QResizeEvent* event) override;

protected Q_SLOTS:
  /*
   * @brief Update the color when button is clicked
   */
  void onButtonClick() override;

private:
  int index_;     // the index of path
  QColor color_;  // the color of path
};
} // namespace rmpv
#endif  // COLOR_EDITOR_H
