/***********************************************************
 *
 * @file: path_visual_plugin.cpp
 * @breif: Contains path visualization Rviz plugin class
 * @author: Yang Haodong, Wu Maojia
 * @update: 2023-10-14
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong, Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "wrapper_planner/CallPlan.h"
#include "include/path_visual_plugin.h"
#include "include/ui_path_visual_plugin.h"
#include "include/core_path_visual_plugin.h"

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_visual_plugin::PathVisualPlugin, rviz::Panel)

namespace path_visual_plugin
{
/**
 * @brief Construct a new Path Visualization Plugin object
 */
PathVisualPlugin::PathVisualPlugin(QWidget* parent) :
  rviz::Panel(parent), ui_(new Ui::PathVisualPlugin), core_(new CorePathVisualPlugin)
{
  ui_->setupUi(this);
  core_->setupROS();
  setupUi();
}

/**
 * @brief Destroy the Path Visualization Plugin object
 */
PathVisualPlugin::~PathVisualPlugin()
{
  if (ui_)
    delete ui_;
  if (core_)
    delete core_;
  if (table_model_)
    delete table_model_;
}

/**
 * @brief User interface parameters initialization
 */
void PathVisualPlugin::setupUi()
{
  table_model_ = new QStandardItemModel();
  table_header_ = QStringList({"Planner", "Start", "Goal", "Length", "Turning Angle", "Color", "Show", "Remove"});
  table_model_->setHorizontalHeaderLabels(table_header_);
  ui_->tableView_list->setModel(table_model_);
  ui_->tableView_list->setColumnWidth(6, 40);

  for (const auto p_name : core_->planner_list_)
    ui_->comboBox_add_planner_global->addItem(QString::fromStdString(p_name));

  _onValueChanged();

  connect(core_, SIGNAL(valueChanged()), this, SLOT(_onValueChanged()));
  connect(ui_->pushButton_add_add, SIGNAL(clicked()), this, SLOT(_onClicked()));
  connect(ui_->lineEdit_add_start_x, SIGNAL(editingFinished()), this, SLOT(_onEditingFinished()));
  connect(ui_->lineEdit_add_start_y, SIGNAL(editingFinished()), this, SLOT(_onEditingFinished()));
  connect(ui_->lineEdit_add_goal_x, SIGNAL(editingFinished()), this, SLOT(_onEditingFinished()));
  connect(ui_->lineEdit_add_goal_y, SIGNAL(editingFinished()), this, SLOT(_onEditingFinished()));
}

/**
   *  @brief if clicked signal is received, call this slot function
 */
void PathVisualPlugin::_onClicked()
{
  QPushButton* senderPushButton = qobject_cast<QPushButton*>(sender());

  if (senderPushButton)
  {
    // get the button name
    QString senderName = senderPushButton->objectName();

    // regular expression to match button names
    QRegularExpression re_color("pushButton_list_color_(\\d+)");
    QRegularExpression re_remove("pushButton_list_remove_(\\d+)");
    QRegularExpressionMatch match_color = re_color.match(senderName);
    QRegularExpressionMatch match_remove = re_remove.match(senderName);

    if (senderName == QString::fromUtf8("pushButton_add_add"))
    {
      core_->addPath(ui_->comboBox_add_planner_global->currentText().toStdString());
//      _addPathRow();
      _updateTableView();
    }
    else if (senderName == QString::fromUtf8("pushButton_files_load"))
      core_->loadPaths();
    else if (senderName == QString::fromUtf8("pushButton_files_save"))
      core_->savePaths();
    else if (match_color.hasMatch())
    {
      QString capturedText = match_color.captured(1);
      bool ok = false;
      int index = capturedText.toInt(&ok);
      if (ok)
      {
        QColor color = QColorDialog::getColor(Qt::darkBlue, this);
        if (color.isValid())
          core_->setPathColor(index, color);
        else
        {
          ROS_ERROR("Invalid color!");
          return;
        }
      }
      else
      {
        ROS_ERROR("Failed to get the path index to set color!");
        return;
      };
    }
    else if (match_remove.hasMatch())
    {
      QString capturedText = match_remove.captured(1);
      bool ok = false;
      int index = capturedText.toInt(&ok);
      if (ok)
      {
        core_->removePath(index);
//        _removePathRow(index);
        _updateTableView();
      }
      else
      {
        ROS_ERROR("Failed to get the path index to remove!");
        return;
      };
    }
    else
    {
      ROS_ERROR("Unknown signal sender QPushButton.");
      return;
    }
  }
  else
    ROS_ERROR("Failed to get signal sender QPushButton.");
}

/**
 *  @brief if editing finished signal is received, call this slot function
 */
void PathVisualPlugin::_onEditingFinished()
{
  // get the signal sender
  QLineEdit* senderLineEdit = qobject_cast<QLineEdit*>(sender());

  if (senderLineEdit)
  {
    QString senderName = senderLineEdit->objectName();
    QString text = senderLineEdit->text();
    bool ok;
    double value = text.toDouble(&ok);
    double* valueToChange;

    if (senderName == QString::fromUtf8("lineEdit_add_start_x"))
      valueToChange = &(core_->start_x_);
    else if (senderName == QString::fromUtf8("lineEdit_add_start_y"))
      valueToChange = &(core_->start_y_);
    else if (senderName == QString::fromUtf8("lineEdit_add_goal_x"))
      valueToChange = &(core_->goal_x_);
    else if (senderName == QString::fromUtf8("lineEdit_add_goal_y"))
      valueToChange = &(core_->goal_y_);
    else
    {
      ROS_ERROR("Unknown signal sender QLineEdit.");
      return;
    }

    if (ok)
      *valueToChange = value;
    senderLineEdit->setText(QString::number(*valueToChange, 'f', 3));
  }
  else
    ROS_ERROR("Failed to get signal sender QLineEdit.");
}

/**
   *  @brief if value changed signal from core is received, call this slot function
 */
void PathVisualPlugin::_onValueChanged()
{
  ui_->lineEdit_add_start_x->setText(QString::number(core_->start_x_, 'f', 3));
  ui_->lineEdit_add_start_y->setText(QString::number(core_->start_y_, 'f', 3));
  ui_->lineEdit_add_goal_x->setText(QString::number(core_->goal_x_, 'f', 3));
  ui_->lineEdit_add_goal_y->setText(QString::number(core_->goal_y_, 'f', 3));
}

/**
   *  @brief add a path row in table view
 */
void PathVisualPlugin::_addPathRow()
{
  int nrow = table_model_->rowCount();
  table_model_->setItem(nrow, 0, new QStandardItem(ui_->comboBox_add_planner_global->currentText()));
  table_model_->setItem(nrow, 1, new QStandardItem(QString("(%1,%2)")
                                                       .arg(QString::number(core_->start_x_, 'f', 3))
                                                       .arg(QString::number(core_->start_y_, 'f', 3))
                                                   ));
  table_model_->setItem(nrow, 2, new QStandardItem(QString("(%1,%2)")
                                                       .arg(QString::number(core_->goal_x_, 'f', 3))
                                                       .arg(QString::number(core_->goal_x_, 'f', 3))
                                                   ));

  QPushButton* pushButton_list_color = new QPushButton();
  pushButton_list_color->setObjectName(QString::fromUtf8("pushButton_list_color_%1")
                                           .arg(QString::number(core_->path_list_->size())));
  pushButton_list_color->setText(QApplication::translate("PathVisualPlugin", "set", nullptr));
  ui_->tableView_list->setIndexWidget(table_model_->index(nrow, 5), pushButton_list_color);
  connect(pushButton_list_color, SIGNAL(clicked()), this, SLOT(_onClicked()));

  QCheckBox* checkBox_list_show = new QCheckBox();
  checkBox_list_show->setAutoFillBackground(true);
  checkBox_list_show->setStyleSheet(
      "background-color: rgb(255, 255, 255);"
//        "padding-left:20px;"
  );
  ui_->tableView_list->setIndexWidget(table_model_->index(nrow, 6), checkBox_list_show);
  checkBox_list_show->setChecked(true);

  QPushButton* pushButton_list_remove = new QPushButton();
  pushButton_list_remove->setObjectName(QString::fromUtf8("pushButton_list_remove_%1")
                                            .arg(QString::number(core_->path_list_->size())));
  pushButton_list_remove->setText(QApplication::translate("PathVisualPlugin", "X", nullptr));
  ui_->tableView_list->setIndexWidget(table_model_->index(nrow, 7), pushButton_list_remove);
  connect(pushButton_list_remove, SIGNAL(clicked()), this, SLOT(_onClicked()));

  ROS_INFO("New path has been successfully added to the path list.");
}

/**
   *  @brief remove a path of some index row in table view
   *  @param index  the index of the removed path
 */
void PathVisualPlugin::_removePathRow(const int& index)
{
  ROS_INFO("The path with index %d has been successfully removed from the path list.", index);
}

void PathVisualPlugin::_updateTableView()
{
  table_model_->clear();
  table_model_->setHorizontalHeaderLabels(table_header_);

  PathInfo row_info;
  for (int row = 0; row < core_->path_list_->size(); ++row)
  {
    bool ok = core_->path_list_->query(row_info, row);
    if (ok)
    {
      table_model_->setItem(row, 0, new QStandardItem(QString::fromStdString(row_info.planner_name)));
      table_model_->setItem(row, 1, new QStandardItem(QString("(%1,%2)")
                                                          .arg(QString::number(row_info.start_x, 'f', 3))
                                                          .arg(QString::number(row_info.start_y, 'f', 3))
                                                      ));
      table_model_->setItem(row, 2, new QStandardItem(QString("(%1,%2)")
                                                          .arg(QString::number(row_info.goal_x, 'f', 3))
                                                          .arg(QString::number(row_info.goal_x, 'f', 3))
                                                      ));

      QPushButton* pushButton_list_color = new QPushButton();
      pushButton_list_color->setObjectName(QString::fromUtf8("pushButton_list_color_%1")
                                               .arg(QString::number(row)));
      pushButton_list_color->setText(QApplication::translate("PathVisualPlugin", "set", nullptr));
      ui_->tableView_list->setIndexWidget(table_model_->index(row, 5), pushButton_list_color);
      connect(pushButton_list_color, SIGNAL(clicked()), this, SLOT(_onClicked()));

      QCheckBox* checkBox_list_show = new QCheckBox();
      checkBox_list_show->setAutoFillBackground(true);
      checkBox_list_show->setStyleSheet(
          "background-color: rgb(255, 255, 255);"
          //        "padding-left:20px;"
      );
      ui_->tableView_list->setIndexWidget(table_model_->index(row, 6), checkBox_list_show);
      checkBox_list_show->setChecked(true);

      QPushButton* pushButton_list_remove = new QPushButton();
      pushButton_list_remove->setObjectName(QString::fromUtf8("pushButton_list_remove_%1")
                                                .arg(QString::number(row)));
      pushButton_list_remove->setText(QApplication::translate("PathVisualPlugin", "X", nullptr));
      ui_->tableView_list->setIndexWidget(table_model_->index(row, 7), pushButton_list_remove);
      connect(pushButton_list_remove, SIGNAL(clicked()), this, SLOT(_onClicked()));
    }
    else
    {
      ROS_ERROR("Failed to update the table view!");
      return;
    }
  }
}
}  // namespace path_visual_plugin
