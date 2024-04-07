/**
 * *********************************************************
 *
 * @file: color_editor.cpp
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
#include "utils/color_editor.h"

namespace rmpv
{
/*
 * @brief Construct a new Color Editor object
 * @param index  the row index of color editor in table model
 * @param color  the color of path
 * @param parent  the parent widget
 */
ColorEditor::ColorEditor(int index, const QColor& color, QWidget* parent)
  : rviz::LineEditWithButton(parent), index_(index), color_(color)
{
  connect(this, &QLineEdit::textChanged, this, &ColorEditor::parseText);
  setColor(color_);
}

/*
 * @brief Slot function to paint just the color box. Paints it in the left end of rect, size rect.height() by
 * rect.height().
 * @param painter  the painter
 * @param rect  the QT rectangle
 * @param color  the color
 */
void ColorEditor::paintColorBox(QPainter* painter, const QRect& rect, const QColor& color)
{
  int padding = 3;
  int size = rect.height() - padding * 2 - 1;
  painter->save();
  painter->setBrush(color);
  painter->drawRoundedRect(rect.x() + padding + 3, rect.y() + padding, size, size, 0, 0, Qt::AbsoluteSize);
  painter->restore();
}

/*
 * @brief Set the color
 * @param color  the new color to set
 */
void ColorEditor::setColor(const QColor& color)
{
  if (color.isValid())
  {
    color_ = color;
    setText(rviz::printColor(color));
    Q_EMIT colorChanged(index_, color_);
  }
  else
    setText(rviz::printColor(color_));
}

/*
 * @brief Parse the text and update the color
 */
void ColorEditor::parseText()
{
  const QString t = text();
  QColor new_color = rviz::parseColor(t);
  setColor(new_color);
}

/*
 * @brief Slot function to call parent version then paint color swatch
 * @param event  the paint event
 */
void ColorEditor::paintEvent(QPaintEvent* event)
{
  LineEditWithButton::paintEvent(event);
  QPainter painter(this);
  painter.setPen(Qt::black);
  paintColorBox(&painter, rect(), color_);
}

/*
 * @brief Resize the color editor
 * @param event  the resize event
 */
void ColorEditor::resizeEvent(QResizeEvent* event)
{
  // Do the normal line-edit-with-button thing
  LineEditWithButton::resizeEvent(event);

  // Then add text padding on the left to make room for the color swatch
  QMargins marge = textMargins();
  setTextMargins(height(), marge.top(), marge.right(), marge.bottom());
}

/*
 * @brief Update the color when button is clicked
 */
void ColorEditor::onButtonClick()
{
  QColor new_color = QColorDialog::getColor(color_, this);
  setColor(new_color);
}
}  // namespace rmpv