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
PathVisualPlugin::PathVisualPlugin(QWidget* parent)
 : rviz::Panel(parent), ui_(new Ui::PathVisualPlugin), core_(new CorePathVisualPlugin)
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
 table_header_ = QStringList({ "Select", "Planner", "Start", "Goal", "Length", "Turning Angle", "Color", "Remove" });
 table_model_->setHorizontalHeaderLabels(table_header_);
 ui_->tableView_list->setModel(table_model_);
 ui_->tableView_list->setItemDelegateForColumn(0, &checkBoxListSelectDelegate_);
 connect(&checkBoxListSelectDelegate_, SIGNAL(checkBoxStateChanged(QModelIndex, bool)), this,
         SLOT(_onCheckBoxStateChanged(QModelIndex, bool)));

 for (const auto p_name : core_->planner_list_)
   ui_->comboBox_add_planner_global->addItem(QString::fromStdString(p_name));

 _onValueChanged();

 connect(core_, SIGNAL(valueChanged()), this, SLOT(_onValueChanged()));
 connect(ui_->pushButton_add_add, SIGNAL(clicked()), this, SLOT(_onClicked()));
 connect(ui_->pushButton_files_load, SIGNAL(clicked()), this, SLOT(_onClicked()));
 connect(ui_->pushButton_files_save, SIGNAL(clicked()), this, SLOT(_onClicked()));
 connect(ui_->lineEdit_add_start_x, SIGNAL(editingFinished()), this, SLOT(_onEditingFinished()));
 connect(ui_->lineEdit_add_start_y, SIGNAL(editingFinished()), this, SLOT(_onEditingFinished()));
 connect(ui_->lineEdit_add_goal_x, SIGNAL(editingFinished()), this, SLOT(_onEditingFinished()));
 connect(ui_->lineEdit_add_goal_y, SIGNAL(editingFinished()), this, SLOT(_onEditingFinished()));
}

/**
*  @brief if clicked signal from pushButton is received, call this slot function
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
     core_->addPath(ui_->comboBox_add_planner_global->currentText());
     _updateTableView();
   }
   else if (senderName == QString::fromUtf8("pushButton_files_load"))
   {
     std::string cur_dir(std::getenv("PWD"));
     QString open_dir = QString::fromStdString(cur_dir + std::string("/../../user_data"));
     QString open_file = QFileDialog::getOpenFileName(this, QStringLiteral("select path files"), open_dir,
                                                      "JSON Files(*.json)", nullptr, QFileDialog::DontResolveSymlinks);
     core_->loadPaths(open_file.toStdString());
     _updateTableView();
   }
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
*  @brief if editing finished signal from lineEdit is received, call this slot function
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
     valueToChange = &(core_->start_.x);
   else if (senderName == QString::fromUtf8("lineEdit_add_start_y"))
     valueToChange = &(core_->start_.y);
   else if (senderName == QString::fromUtf8("lineEdit_add_goal_x"))
     valueToChange = &(core_->goal_.x);
   else if (senderName == QString::fromUtf8("lineEdit_add_goal_y"))
     valueToChange = &(core_->goal_.y);
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
*  @brief if state changed signal is from checkBox received, call this slot function
*/
void PathVisualPlugin::_onCheckBoxStateChanged(const QModelIndex &index, const bool &checked)
{
 core_->setPathSelectStatus(index.row(), checked);
 ROS_WARN("index: %d, checked: %d", index.row(), checked);
}
void PathVisualPlugin::_onStateChanged(int state)
{
 QCheckBox* senderCheckBox = qobject_cast<QCheckBox*>(sender());

 if (senderCheckBox)
 {
   // get the button name
   QString senderName = senderCheckBox->objectName();
//   ROS_WARN("senderName: %s", senderName.toStdString());

   // regular expression to match button names
   QRegularExpression re("checkBox_list_select_(\\d+)");
   QRegularExpressionMatch match = re.match(senderName);

   if (match.hasMatch())
   {
     QString capturedText = match.captured(1);
     bool ok = false;
     int index = capturedText.toInt(&ok);
     if (ok)
       core_->setPathSelectStatus(index, (state == 0) ? false : true);
     else
     {
       ROS_ERROR("Failed to get the path index to set color!");
       return;
     };
   }
   else
   {
     ROS_ERROR("Unknown signal sender QCheckBox.");
     return;
   }
 }
 else
   ROS_ERROR("Failed to get signal sender QCheckBox.");
}

/**
*  @brief if value changed signal from core is received, call this slot function
*/
void PathVisualPlugin::_onValueChanged()
{
 ui_->lineEdit_add_start_x->setText(QString::number(core_->start_.x, 'f', 3));
 ui_->lineEdit_add_start_y->setText(QString::number(core_->start_.y, 'f', 3));
 ui_->lineEdit_add_goal_x->setText(QString::number(core_->goal_.x, 'f', 3));
 ui_->lineEdit_add_goal_y->setText(QString::number(core_->goal_.y, 'f', 3));
}

void PathVisualPlugin::_updateTableView()
{
 // initialize table model
 table_model_->clear();
 table_model_->setHorizontalHeaderLabels(table_header_);
 table_model_->setRowCount(core_->path_list_->size());

 PathInfo row_info;
 for (int row = 0; row < core_->path_list_->size(); ++row)
 {
   bool ok = core_->path_list_->query(row_info, row);
   if (ok)
   {
//     ROS_WARN("debugging");
//     ROS_WARN(checkBoxListSelectDelegate_.editors.isEmpty() ? "is empty" : "not empty");
//     // 0th column: select status
//     QHash<QModelIndex, QCheckBox*>::const_iterator it;
//     for (it = checkBoxListSelectDelegate_.editors.constBegin(); it != checkBoxListSelectDelegate_.editors.constEnd(); ++it) {
//       const QModelIndex &key = it.key();
//       ROS_WARN("key: (%d, %d)", key.row(), key.column());
//     }
//     QCheckBox* checkBox_list_select = checkBoxListSelectDelegate_.getEditor(table_model_->index(row, 0));
//     connect(checkBox_list_select, SIGNAL(stateChanged(int)), this, SLOT(_onStateChanged(int)));
     table_model_->setData(table_model_->index(row, 0), row_info.select, Qt::UserRole);

     // 1st column: planner name
     table_model_->setItem(row, 1, new QStandardItem(row_info.getData(PathInfo::plannerName).toString()));

     // 2nd column: start point
     table_model_->setItem(row, 2, new QStandardItem(QString("(%1,%2)")
                                                         .arg(QString::number(row_info.getData(PathInfo::startPointX).toDouble(), 'f', 3))
                                                         .arg(QString::number(row_info.getData(PathInfo::startPointY).toDouble(), 'f', 3))
                                                     ));

     // 3rd column: goal point
     table_model_->setItem(row, 3, new QStandardItem(QString("(%1,%2)")
                                                         .arg(QString::number(row_info.getData(PathInfo::goalPointX).toDouble(), 'f', 3))
                                                         .arg(QString::number(row_info.getData(PathInfo::goalPointY).toDouble(), 'f', 3))
                                                     ));

     // 6th column: color
     QPushButton* pushButton_list_color = new QPushButton();
     pushButton_list_color->setObjectName(QString::fromUtf8("pushButton_list_color_%1").arg(QString::number(row)));
     pushButton_list_color->setText(QApplication::translate("PathVisualPlugin", "set", nullptr));
     ui_->tableView_list->setIndexWidget(table_model_->index(row, 6), pushButton_list_color);
     connect(pushButton_list_color, SIGNAL(clicked()), this, SLOT(_onClicked()));

     // 7th column: remove button
     QPushButton* pushButton_list_remove = new QPushButton();
     pushButton_list_remove->setObjectName(QString::fromUtf8("pushButton_list_remove_%1").arg(QString::number(row)));
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