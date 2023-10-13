#ifndef TABLE_DELEGATE_H
#define TABLE_DELEGATE_H
namespace path_visual_plugin
{

class CheckBoxDelegate : public QStyledItemDelegate
{
public:
  CheckBoxDelegate(QObject *parent = nullptr) : QStyledItemDelegate(parent) {}

  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;

  QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};
#endif  // TABLE_DELEGATE_H
