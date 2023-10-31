/***********************************************************
*
* @file: table_delegates.h
* @breif: Contains delegate classes for table view
* @author: Yang Haodong, Wu Maojia
* @update: 2023-10-30
* @version: 1.0
*
* Copyright (c) 2023， Yang Haodong, Wu Maojia
* All rights reserved.
* --------------------------------------------------------
*
**********************************************************/
#ifndef TABLE_DELEGATE_H
#define TABLE_DELEGATE_H

#include <QtWidgets>
#include <QStyledItemDelegate>

namespace path_visual_plugin
{
class CheckBoxListSelectDelegate : public QStyledItemDelegate
{
  Q_OBJECT

public:
  QWidget* createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;

  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;

  bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) override;

Q_SIGNALS:
  void selectStateChanged(const QModelIndex &index, const bool &checked);
};
} // namespace path_visual_plugin
#endif  // TABLE_DELEGATE_H
