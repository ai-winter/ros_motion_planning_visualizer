/********************************************************************************
** Form generated from reading UI file 'rmpv.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RMPV_H
#define UI_RMPV_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RMPV
{
public:
    QVBoxLayout *verticalLayout_main;
    QTabWidget *tabManager;
    QWidget *tab_path;
    QGridLayout *gridLayout_path;
    QGroupBox *groupBox_path_add;
    QVBoxLayout *verticalLayout_path_add;
    QGroupBox *groupBox_path_add_start;
    QHBoxLayout *horizontalLayout_path_add_start;
    QLabel *label_path_add_start_x;
    QLineEdit *lineEdit_path_add_start_x;
    QLabel *label_path_add_start_y;
    QLineEdit *lineEdit_path_add_start_y;
    QLabel *label_path_add_start_theta;
    QLineEdit *lineEdit_path_add_start_theta;
    QGroupBox *groupBox_path_add_goal;
    QHBoxLayout *horizontalLayout_path_add_goal;
    QLabel *label_path_add_goal_x;
    QLineEdit *lineEdit_path_add_goal_x;
    QLabel *label_path_add_goal_y;
    QLineEdit *lineEdit_path_add_goal_y;
    QLabel *label_path_add_goal_theta;
    QLineEdit *lineEdit_path_add_goal_theta;
    QGroupBox *groupBox_path_add_planner;
    QHBoxLayout *horizontalLayout_path_add_planner;
    QLabel *label_path_add_planner_global;
    QComboBox *comboBox_path_add_planner_global;
    QSpacerItem *verticalSpacer_path_add_1;
    QPushButton *pushButton_path_add_add;
    QSpacerItem *verticalSpacer_path_add_2;
    QGroupBox *groupBox_path_list;
    QHBoxLayout *horizontalLayout_path_list;
    QTableView *tableView_path_list;
    QGroupBox *groupBox_path_files;
    QVBoxLayout *verticalLayout_path_files;
    QSpacerItem *verticalSpacer_path_files_1;
    QPushButton *pushButton_path_files_load;
    QSpacerItem *verticalSpacer_path_files_2;
    QPushButton *pushButton_path_files_save;
    QSpacerItem *verticalSpacer_path_files_3;

    void setupUi(QWidget *RMPV)
    {
        if (RMPV->objectName().isEmpty())
            RMPV->setObjectName(QString::fromUtf8("RMPV"));
        RMPV->resize(800, 600);
        verticalLayout_main = new QVBoxLayout(RMPV);
        verticalLayout_main->setObjectName(QString::fromUtf8("verticalLayout_main"));
        tabManager = new QTabWidget(RMPV);
        tabManager->setObjectName(QString::fromUtf8("tabManager"));
        tabManager->setMinimumSize(QSize(100, 0));
        tabManager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_path = new QWidget();
        tab_path->setObjectName(QString::fromUtf8("tab_path"));
        gridLayout_path = new QGridLayout(tab_path);
        gridLayout_path->setObjectName(QString::fromUtf8("gridLayout_path"));
        groupBox_path_add = new QGroupBox(tab_path);
        groupBox_path_add->setObjectName(QString::fromUtf8("groupBox_path_add"));
        groupBox_path_add->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_path_add->sizePolicy().hasHeightForWidth());
        groupBox_path_add->setSizePolicy(sizePolicy);
        groupBox_path_add->setMaximumSize(QSize(325, 16777215));
        verticalLayout_path_add = new QVBoxLayout(groupBox_path_add);
        verticalLayout_path_add->setObjectName(QString::fromUtf8("verticalLayout_path_add"));
        groupBox_path_add_start = new QGroupBox(groupBox_path_add);
        groupBox_path_add_start->setObjectName(QString::fromUtf8("groupBox_path_add_start"));
        horizontalLayout_path_add_start = new QHBoxLayout(groupBox_path_add_start);
        horizontalLayout_path_add_start->setObjectName(QString::fromUtf8("horizontalLayout_path_add_start"));
        label_path_add_start_x = new QLabel(groupBox_path_add_start);
        label_path_add_start_x->setObjectName(QString::fromUtf8("label_path_add_start_x"));

        horizontalLayout_path_add_start->addWidget(label_path_add_start_x);

        lineEdit_path_add_start_x = new QLineEdit(groupBox_path_add_start);
        lineEdit_path_add_start_x->setObjectName(QString::fromUtf8("lineEdit_path_add_start_x"));

        horizontalLayout_path_add_start->addWidget(lineEdit_path_add_start_x);

        label_path_add_start_y = new QLabel(groupBox_path_add_start);
        label_path_add_start_y->setObjectName(QString::fromUtf8("label_path_add_start_y"));

        horizontalLayout_path_add_start->addWidget(label_path_add_start_y);

        lineEdit_path_add_start_y = new QLineEdit(groupBox_path_add_start);
        lineEdit_path_add_start_y->setObjectName(QString::fromUtf8("lineEdit_path_add_start_y"));

        horizontalLayout_path_add_start->addWidget(lineEdit_path_add_start_y);

        label_path_add_start_theta = new QLabel(groupBox_path_add_start);
        label_path_add_start_theta->setObjectName(QString::fromUtf8("label_path_add_start_theta"));

        horizontalLayout_path_add_start->addWidget(label_path_add_start_theta);

        lineEdit_path_add_start_theta = new QLineEdit(groupBox_path_add_start);
        lineEdit_path_add_start_theta->setObjectName(QString::fromUtf8("lineEdit_path_add_start_theta"));

        horizontalLayout_path_add_start->addWidget(lineEdit_path_add_start_theta);


        verticalLayout_path_add->addWidget(groupBox_path_add_start);

        groupBox_path_add_goal = new QGroupBox(groupBox_path_add);
        groupBox_path_add_goal->setObjectName(QString::fromUtf8("groupBox_path_add_goal"));
        horizontalLayout_path_add_goal = new QHBoxLayout(groupBox_path_add_goal);
        horizontalLayout_path_add_goal->setObjectName(QString::fromUtf8("horizontalLayout_path_add_goal"));
        label_path_add_goal_x = new QLabel(groupBox_path_add_goal);
        label_path_add_goal_x->setObjectName(QString::fromUtf8("label_path_add_goal_x"));

        horizontalLayout_path_add_goal->addWidget(label_path_add_goal_x);

        lineEdit_path_add_goal_x = new QLineEdit(groupBox_path_add_goal);
        lineEdit_path_add_goal_x->setObjectName(QString::fromUtf8("lineEdit_path_add_goal_x"));

        horizontalLayout_path_add_goal->addWidget(lineEdit_path_add_goal_x);

        label_path_add_goal_y = new QLabel(groupBox_path_add_goal);
        label_path_add_goal_y->setObjectName(QString::fromUtf8("label_path_add_goal_y"));

        horizontalLayout_path_add_goal->addWidget(label_path_add_goal_y);

        lineEdit_path_add_goal_y = new QLineEdit(groupBox_path_add_goal);
        lineEdit_path_add_goal_y->setObjectName(QString::fromUtf8("lineEdit_path_add_goal_y"));

        horizontalLayout_path_add_goal->addWidget(lineEdit_path_add_goal_y);

        label_path_add_goal_theta = new QLabel(groupBox_path_add_goal);
        label_path_add_goal_theta->setObjectName(QString::fromUtf8("label_path_add_goal_theta"));

        horizontalLayout_path_add_goal->addWidget(label_path_add_goal_theta);

        lineEdit_path_add_goal_theta = new QLineEdit(groupBox_path_add_goal);
        lineEdit_path_add_goal_theta->setObjectName(QString::fromUtf8("lineEdit_path_add_goal_theta"));

        horizontalLayout_path_add_goal->addWidget(lineEdit_path_add_goal_theta);


        verticalLayout_path_add->addWidget(groupBox_path_add_goal);

        groupBox_path_add_planner = new QGroupBox(groupBox_path_add);
        groupBox_path_add_planner->setObjectName(QString::fromUtf8("groupBox_path_add_planner"));
        horizontalLayout_path_add_planner = new QHBoxLayout(groupBox_path_add_planner);
        horizontalLayout_path_add_planner->setObjectName(QString::fromUtf8("horizontalLayout_path_add_planner"));
        label_path_add_planner_global = new QLabel(groupBox_path_add_planner);
        label_path_add_planner_global->setObjectName(QString::fromUtf8("label_path_add_planner_global"));

        horizontalLayout_path_add_planner->addWidget(label_path_add_planner_global);

        comboBox_path_add_planner_global = new QComboBox(groupBox_path_add_planner);
        comboBox_path_add_planner_global->setObjectName(QString::fromUtf8("comboBox_path_add_planner_global"));

        horizontalLayout_path_add_planner->addWidget(comboBox_path_add_planner_global);


        verticalLayout_path_add->addWidget(groupBox_path_add_planner);

        verticalSpacer_path_add_1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_path_add->addItem(verticalSpacer_path_add_1);

        pushButton_path_add_add = new QPushButton(groupBox_path_add);
        pushButton_path_add_add->setObjectName(QString::fromUtf8("pushButton_path_add_add"));

        verticalLayout_path_add->addWidget(pushButton_path_add_add);

        verticalSpacer_path_add_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_path_add->addItem(verticalSpacer_path_add_2);


        gridLayout_path->addWidget(groupBox_path_add, 0, 2, 2, 1);

        groupBox_path_list = new QGroupBox(tab_path);
        groupBox_path_list->setObjectName(QString::fromUtf8("groupBox_path_list"));
        sizePolicy.setHeightForWidth(groupBox_path_list->sizePolicy().hasHeightForWidth());
        groupBox_path_list->setSizePolicy(sizePolicy);
        horizontalLayout_path_list = new QHBoxLayout(groupBox_path_list);
        horizontalLayout_path_list->setObjectName(QString::fromUtf8("horizontalLayout_path_list"));
        tableView_path_list = new QTableView(groupBox_path_list);
        tableView_path_list->setObjectName(QString::fromUtf8("tableView_path_list"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(tableView_path_list->sizePolicy().hasHeightForWidth());
        tableView_path_list->setSizePolicy(sizePolicy1);
        tableView_path_list->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        tableView_path_list->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        tableView_path_list->setEditTriggers(QAbstractItemView::NoEditTriggers);

        horizontalLayout_path_list->addWidget(tableView_path_list);


        gridLayout_path->addWidget(groupBox_path_list, 0, 0, 3, 2);

        groupBox_path_files = new QGroupBox(tab_path);
        groupBox_path_files->setObjectName(QString::fromUtf8("groupBox_path_files"));
        groupBox_path_files->setMaximumSize(QSize(325, 16777215));
        verticalLayout_path_files = new QVBoxLayout(groupBox_path_files);
        verticalLayout_path_files->setObjectName(QString::fromUtf8("verticalLayout_path_files"));
        verticalSpacer_path_files_1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_path_files->addItem(verticalSpacer_path_files_1);

        pushButton_path_files_load = new QPushButton(groupBox_path_files);
        pushButton_path_files_load->setObjectName(QString::fromUtf8("pushButton_path_files_load"));

        verticalLayout_path_files->addWidget(pushButton_path_files_load);

        verticalSpacer_path_files_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_path_files->addItem(verticalSpacer_path_files_2);

        pushButton_path_files_save = new QPushButton(groupBox_path_files);
        pushButton_path_files_save->setObjectName(QString::fromUtf8("pushButton_path_files_save"));

        verticalLayout_path_files->addWidget(pushButton_path_files_save);

        verticalSpacer_path_files_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_path_files->addItem(verticalSpacer_path_files_3);


        gridLayout_path->addWidget(groupBox_path_files, 2, 2, 1, 1);

        tabManager->addTab(tab_path, QString());

        verticalLayout_main->addWidget(tabManager);

        QWidget::setTabOrder(tabManager, tableView_path_list);
        QWidget::setTabOrder(tableView_path_list, lineEdit_path_add_start_x);
        QWidget::setTabOrder(lineEdit_path_add_start_x, lineEdit_path_add_start_y);
        QWidget::setTabOrder(lineEdit_path_add_start_y, lineEdit_path_add_start_theta);
        QWidget::setTabOrder(lineEdit_path_add_start_theta, lineEdit_path_add_goal_x);
        QWidget::setTabOrder(lineEdit_path_add_goal_x, lineEdit_path_add_goal_y);
        QWidget::setTabOrder(lineEdit_path_add_goal_y, lineEdit_path_add_goal_theta);
        QWidget::setTabOrder(lineEdit_path_add_goal_theta, comboBox_path_add_planner_global);
        QWidget::setTabOrder(comboBox_path_add_planner_global, pushButton_path_add_add);
        QWidget::setTabOrder(pushButton_path_add_add, pushButton_path_files_load);
        QWidget::setTabOrder(pushButton_path_files_load, pushButton_path_files_save);

        retranslateUi(RMPV);

        tabManager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(RMPV);
    } // setupUi

    void retranslateUi(QWidget *RMPV)
    {
        RMPV->setWindowTitle(QCoreApplication::translate("RMPV", "RMPV - ROS Motion Planning Visualizer", nullptr));
#if QT_CONFIG(tooltip)
        groupBox_path_add->setToolTip(QString());
#endif // QT_CONFIG(tooltip)
        groupBox_path_add->setTitle(QCoreApplication::translate("RMPV", "Add a New Path", nullptr));
#if QT_CONFIG(tooltip)
        groupBox_path_add_start->setToolTip(QCoreApplication::translate("RMPV", "The start pose of the new path", nullptr));
#endif // QT_CONFIG(tooltip)
        groupBox_path_add_start->setTitle(QCoreApplication::translate("RMPV", "Start Pose", nullptr));
        label_path_add_start_x->setText(QCoreApplication::translate("RMPV", "x", nullptr));
        label_path_add_start_y->setText(QCoreApplication::translate("RMPV", "y", nullptr));
        label_path_add_start_theta->setText(QCoreApplication::translate("RMPV", "\316\270", nullptr));
#if QT_CONFIG(tooltip)
        groupBox_path_add_goal->setToolTip(QCoreApplication::translate("RMPV", "The goal pose of the new path", nullptr));
#endif // QT_CONFIG(tooltip)
        groupBox_path_add_goal->setTitle(QCoreApplication::translate("RMPV", "Goal Pose", nullptr));
        label_path_add_goal_x->setText(QCoreApplication::translate("RMPV", "x", nullptr));
        label_path_add_goal_y->setText(QCoreApplication::translate("RMPV", "y", nullptr));
        label_path_add_goal_theta->setText(QCoreApplication::translate("RMPV", "\316\270", nullptr));
#if QT_CONFIG(tooltip)
        groupBox_path_add_planner->setToolTip(QCoreApplication::translate("RMPV", "The planner used to plan the new path", nullptr));
#endif // QT_CONFIG(tooltip)
        groupBox_path_add_planner->setTitle(QCoreApplication::translate("RMPV", "Planner", nullptr));
#if QT_CONFIG(tooltip)
        label_path_add_planner_global->setToolTip(QString());
#endif // QT_CONFIG(tooltip)
        label_path_add_planner_global->setText(QCoreApplication::translate("RMPV", "Global Planner", nullptr));
#if QT_CONFIG(tooltip)
        comboBox_path_add_planner_global->setToolTip(QString());
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        pushButton_path_add_add->setToolTip(QCoreApplication::translate("RMPV", "Plan and add the configured path to the path list", nullptr));
#endif // QT_CONFIG(tooltip)
        pushButton_path_add_add->setText(QCoreApplication::translate("RMPV", "Start Planning", nullptr));
#if QT_CONFIG(tooltip)
        groupBox_path_list->setToolTip(QString());
#endif // QT_CONFIG(tooltip)
        groupBox_path_list->setTitle(QCoreApplication::translate("RMPV", "Path List", nullptr));
#if QT_CONFIG(tooltip)
        groupBox_path_files->setToolTip(QString());
#endif // QT_CONFIG(tooltip)
        groupBox_path_files->setTitle(QCoreApplication::translate("RMPV", "For Path Files", nullptr));
#if QT_CONFIG(tooltip)
        pushButton_path_files_load->setToolTip(QCoreApplication::translate("RMPV", "Load some .json path data files and append them to the path list", nullptr));
#endif // QT_CONFIG(tooltip)
        pushButton_path_files_load->setText(QCoreApplication::translate("RMPV", "Load Paths", nullptr));
#if QT_CONFIG(tooltip)
        pushButton_path_files_save->setToolTip(QCoreApplication::translate("RMPV", "Save selected paths data into a .json file", nullptr));
#endif // QT_CONFIG(tooltip)
        pushButton_path_files_save->setText(QCoreApplication::translate("RMPV", "Save Paths", nullptr));
        tabManager->setTabText(tabManager->indexOf(tab_path), QCoreApplication::translate("RMPV", "Path Visualizer", nullptr));
    } // retranslateUi

};

namespace Ui {
    class RMPV: public Ui_RMPV {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RMPV_H
