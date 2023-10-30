/***********************************************************
*
* @file: table_delegate.cpp
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
#include <QDebug>
#include "include/table_delegate.h"

namespace path_visual_plugin
{
QWidget* CheckBoxListSelectDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                                                  const QModelIndex &index) const
{
    QCheckBox* editor = new QCheckBox();
    editor->setTristate(false);
    return editor;
}

void CheckBoxListSelectDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    if (index.column() == 0)
    {
      bool value = index.data(Qt::UserRole).toInt();
      QStyleOptionButton checkBoxOption;
      QRect checkBoxRect = QApplication::style()->subElementRect(QStyle::SE_CheckBoxIndicator, &checkBoxOption);
      checkBoxOption.rect = option.rect;
      checkBoxOption.rect.setLeft(option.rect.left() + (option.rect.width() - checkBoxRect.width()) / 2);
      checkBoxOption.rect.setTop(option.rect.top() + (option.rect.height() - checkBoxRect.height()) / 2);
      checkBoxOption.state = value ? QStyle::State_On : QStyle::State_Off;
      checkBoxOption.state |= QStyle::State_Enabled;
      QApplication::style()->drawControl(QStyle::CE_CheckBox, &checkBoxOption, painter);
    }
    else
      QStyledItemDelegate::paint(painter,option,index);
}

bool CheckBoxListSelectDelegate::editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index)
{
    // disable double clicks and  releases
    if (event->type() == QEvent::MouseButtonDblClick || event->type() == QEvent::MouseButtonRelease)
      return false;

    bool checked = index.data(Qt::UserRole).toBool();
    model->setData(index, !checked, Qt::UserRole);
    Q_EMIT checkBoxStateChanged(index, !checked);

    return QStyledItemDelegate::editorEvent(event,model,option,index);
}
} // namespace path_visual_plugin