namespace path_visual_plugin
{
  void CheckBoxDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override
  {
    QStyleOptionViewItem checkBoxOption(option);
    initStyleOption(&checkBoxOption, index);

    // 计算复选框的位置使其居中
    int x = option.rect.center().x() - checkBoxOption.rect.width() / 2;
    int y = option.rect.center().y() - checkBoxOption.rect.height() / 2;
    checkBoxOption.rect.moveTo(x, y);

    QCheckBox checkBox;
    checkBox.setChecked(index.data(Qt::CheckStateRole).toBool());
    checkBox.setStyle(option.widget ? option.widget->style() : QApplication::style());
    checkBox.setAttribute(Qt::WA_TransparentForMouseEvents);
    checkBox.render(painter, checkBoxOption.rect.topLeft());
  }

  QSize CheckBoxDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override
  {
    QStyleOptionViewItem checkBoxOption(option);
    initStyleOption(&checkBoxOption, index);
    return QSize(checkBoxOption.rect.width(), checkBoxOption.rect.height());
  }
};