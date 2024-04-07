/**
 * *********************************************************
 *
 * @file: select_delegate.h
 * @brief: Contains select column delegate classes for table view
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
#include "utils/select_delegate.h"

namespace rmpv
{
/*
 * @brief Create a new checkBox in a cell in the table model
 * @param parent  the parent widget used to control how the editor widget appears
 * @param option  the style option used to control how the editor widget appears
 * @param index   cell index in model
 * @return  the widget used to edit the item specified by index for editing
 */
QWidget* selectDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                                      const QModelIndex& index) const
{
  QCheckBox* editor = new QCheckBox(parent);
  editor->setTristate(false);
  return editor;
}

/*
 * @brief Renders the delegate using the given painter and style option for the item specified by index
 * @param painter  the painter used to paint the item
 * @param option   used to control how the item is rendered
 * @param index    cell index in model
 */
void selectDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
  bool value = index.data(Qt::UserRole).toBool();
  QStyleOptionButton checkBoxOption;
  QRect checkBoxRect = QApplication::style()->subElementRect(QStyle::SE_CheckBoxIndicator, &checkBoxOption);
  checkBoxOption.rect = option.rect;
  checkBoxOption.rect.setLeft(option.rect.left() + (option.rect.width() - checkBoxRect.width()) / 2);
  checkBoxOption.state = value ? QStyle::State_On : QStyle::State_Off;
  checkBoxOption.state |= QStyle::State_Enabled;
  QApplication::style()->drawControl(QStyle::CE_CheckBox, &checkBoxOption, painter);
}

/*
 * @brief Processes the user's interaction with the item specified by index
 * @param event   the event that occurred
 * @param model   the model that contains the item
 * @param option  used to control how the item is rendered
 * @param index   cell index in model
 * @return  true if the data was successfully updated; otherwise returns false
 */
bool selectDelegate::editorEvent(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option,
                                 const QModelIndex& index)
{
  // disable double clicks and  releases
  if (event->type() == QEvent::MouseButtonDblClick || event->type() == QEvent::MouseButtonRelease)
    return false;

  bool checked = index.data(Qt::UserRole).toBool();
  model->setData(index, !checked, Qt::UserRole);
  Q_EMIT selectChanged(index.row(), !checked);

  return QStyledItemDelegate::editorEvent(event, model, option, index);
}
}  // namespace rmpv