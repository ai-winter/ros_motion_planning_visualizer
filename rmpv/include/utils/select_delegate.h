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
#ifndef TABLE_DELEGATE_H
#define TABLE_DELEGATE_H

#include <QtWidgets>
#include <QStyledItemDelegate>

namespace rmpv
{
class selectDelegate : public QStyledItemDelegate
{
  Q_OBJECT

public:
  /*
   * @brief Create a new checkBox in a cell in the table model
   * @param parent  the parent widget used to control how the editor widget appears
   * @param option  the style option used to control how the editor widget appears
   * @param index   cell index in model
   * @return  the widget used to edit the item specified by index for editing
   */
  QWidget* createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;

  /*
   * @brief Renders the delegate using the given painter and style option for the item specified by index
   * @param painter  the painter used to paint the item
   * @param option   used to control how the item is rendered
   * @param index    cell index in model
   */
  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;

  /*
   * @brief Processes the user's interaction with the item specified by index
   * @param event   the event that occurred
   * @param model   the model that contains the item
   * @param option  used to control how the item is rendered
   * @param index   cell index in model
   * @return  true if the data was successfully updated; otherwise returns false
   */
  bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) override;

Q_SIGNALS:
  /*
   * @brief Signal emitted when the user changes the check state of the editor
   * @param index   cell index in model
   * @param checked the new check state of the editor
   */
  void selectChanged(const int &index, const bool &checked);
};
} // namespace rmpv
#endif  // TABLE_DELEGATE_H
