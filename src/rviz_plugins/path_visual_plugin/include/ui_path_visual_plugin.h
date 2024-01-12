/********************************************************************************
** Form generated from reading UI file 'path_visual_plugin.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PATH_VISUAL_PLUGIN_H
#define UI_PATH_VISUAL_PLUGIN_H

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

class Ui_PathVisualPlugin
{
public:
    QVBoxLayout *verticalLayout_main;
    QTabWidget *tabManager;
    QWidget *tab_path;
    QGridLayout *gridLayout_pathVisualization;
    QGroupBox *groupBox_path_list;
    QHBoxLayout *horizontalLayout_list;
    QTableView *tableView_path_list;
    QGroupBox *groupBox_path_add;
    QVBoxLayout *verticalLayout_add;
    QGroupBox *groupBox_path_add_start;
    QHBoxLayout *horizontalLayout_add_start;
    QLabel *label_path_add_start_x;
    QLineEdit *lineEdit_path_add_start_x;
    QLabel *label_path_add_start_y;
    QLineEdit *lineEdit_path_add_start_y;
    QLabel *label_path_add_start_yaw;
    QLineEdit *lineEdit_path_add_start_yaw;
    QGroupBox *groupBox_path_add_goal;
    QHBoxLayout *horizontalLayout_add_goal;
    QLabel *label_path_add_goal_x;
    QLineEdit *lineEdit_path_add_goal_x;
    QLabel *label_path_add_goal_y;
    QLineEdit *lineEdit_path_add_goal_y;
    QLabel *label_path_add_goal_yaw;
    QLineEdit *lineEdit_path_add_goal_yaw;
    QGroupBox *groupBox_path_add_planner;
    QHBoxLayout *horizontalLayout_add_planner;
    QLabel *label_path_add_planner_global;
    QComboBox *comboBox_path_add_planner_global;
    QSpacerItem *verticalSpacer_path_add_1;
    QPushButton *pushButton_path_add_add;
    QSpacerItem *verticalSpacer_path_add_2;
    QGroupBox *groupBox_path_files;
    QVBoxLayout *verticalLayout_files;
    QSpacerItem *verticalSpacer_path_files_1;
    QPushButton *pushButton_path_files_load;
    QSpacerItem *verticalSpacer_path_files_2;
    QPushButton *pushButton_path_files_save;
    QSpacerItem *verticalSpacer_path_files_3;
    QWidget *tab_curve;
    QGridLayout *gridLayout;
    QGroupBox *groupBox_curve_curves;
    QHBoxLayout *horizontalLayout_list_2;
    QTableView *tableView_curve_curves;
    QGroupBox *groupBox_curve_files;
    QVBoxLayout *verticalLayout_files_2;
    QSpacerItem *verticalSpacer_curve_files_1;
    QPushButton *pushButton_curve_files_load;
    QSpacerItem *verticalSpacer_curve_files_2;
    QPushButton *pushButton_curve_files_save;
    QSpacerItem *verticalSpacer_curve_files_3;
    QGroupBox *groupBox_curve_poses;
    QHBoxLayout *horizontalLayout_list_3;
    QTableView *tableView_curve_poses;
    QGroupBox *groupBox_curve_addPose;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout_curve_addPose;
    QLabel *label_curve_addPose_x;
    QLabel *label_curve_addPose_y;
    QLineEdit *lineEdit_curve_addPose_yaw;
    QLineEdit *lineEdit_curve_addPose_x;
    QLabel *label_curve_addPose_yaw;
    QLineEdit *lineEdit_curve_addPose_y;
    QSpacerItem *verticalSpacer_curve_addPose_1;
    QPushButton *pushButton_curve_addPose_add;
    QSpacerItem *verticalSpacer_curve_addPose_2;
    QGroupBox *groupBox_curve_addCurve;
    QVBoxLayout *verticalLayout_add_2;
    QHBoxLayout *horizontalLayout_curve_addCurve_type;
    QLabel *label_curve_addCurve_type;
    QComboBox *comboBox_curve_addCurve_type;
    QSpacerItem *verticalSpacer_curve_addCurve_1;
    QPushButton *pushButton_curve_addCurve_add;
    QSpacerItem *verticalSpacer_curve_addCurve_2;

    void setupUi(QWidget *PathVisualPlugin)
    {
        if (PathVisualPlugin->objectName().isEmpty())
            PathVisualPlugin->setObjectName(QString::fromUtf8("PathVisualPlugin"));
        PathVisualPlugin->resize(800, 600);
        verticalLayout_main = new QVBoxLayout(PathVisualPlugin);
        verticalLayout_main->setObjectName(QString::fromUtf8("verticalLayout_main"));
        tabManager = new QTabWidget(PathVisualPlugin);
        tabManager->setObjectName(QString::fromUtf8("tabManager"));
        tabManager->setMinimumSize(QSize(100, 0));
        tabManager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_path = new QWidget();
        tab_path->setObjectName(QString::fromUtf8("tab_path"));
        gridLayout_pathVisualization = new QGridLayout(tab_path);
        gridLayout_pathVisualization->setObjectName(QString::fromUtf8("gridLayout_pathVisualization"));
        groupBox_path_list = new QGroupBox(tab_path);
        groupBox_path_list->setObjectName(QString::fromUtf8("groupBox_path_list"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_path_list->sizePolicy().hasHeightForWidth());
        groupBox_path_list->setSizePolicy(sizePolicy);
        horizontalLayout_list = new QHBoxLayout(groupBox_path_list);
        horizontalLayout_list->setObjectName(QString::fromUtf8("horizontalLayout_list"));
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

        horizontalLayout_list->addWidget(tableView_path_list);


        gridLayout_pathVisualization->addWidget(groupBox_path_list, 0, 0, 3, 2);

        groupBox_path_add = new QGroupBox(tab_path);
        groupBox_path_add->setObjectName(QString::fromUtf8("groupBox_path_add"));
        groupBox_path_add->setEnabled(true);
        sizePolicy.setHeightForWidth(groupBox_path_add->sizePolicy().hasHeightForWidth());
        groupBox_path_add->setSizePolicy(sizePolicy);
        groupBox_path_add->setMaximumSize(QSize(350, 16777215));
        verticalLayout_add = new QVBoxLayout(groupBox_path_add);
        verticalLayout_add->setObjectName(QString::fromUtf8("verticalLayout_add"));
        groupBox_path_add_start = new QGroupBox(groupBox_path_add);
        groupBox_path_add_start->setObjectName(QString::fromUtf8("groupBox_path_add_start"));
        horizontalLayout_add_start = new QHBoxLayout(groupBox_path_add_start);
        horizontalLayout_add_start->setObjectName(QString::fromUtf8("horizontalLayout_add_start"));
        label_path_add_start_x = new QLabel(groupBox_path_add_start);
        label_path_add_start_x->setObjectName(QString::fromUtf8("label_path_add_start_x"));

        horizontalLayout_add_start->addWidget(label_path_add_start_x);

        lineEdit_path_add_start_x = new QLineEdit(groupBox_path_add_start);
        lineEdit_path_add_start_x->setObjectName(QString::fromUtf8("lineEdit_path_add_start_x"));

        horizontalLayout_add_start->addWidget(lineEdit_path_add_start_x);

        label_path_add_start_y = new QLabel(groupBox_path_add_start);
        label_path_add_start_y->setObjectName(QString::fromUtf8("label_path_add_start_y"));

        horizontalLayout_add_start->addWidget(label_path_add_start_y);

        lineEdit_path_add_start_y = new QLineEdit(groupBox_path_add_start);
        lineEdit_path_add_start_y->setObjectName(QString::fromUtf8("lineEdit_path_add_start_y"));

        horizontalLayout_add_start->addWidget(lineEdit_path_add_start_y);

        label_path_add_start_yaw = new QLabel(groupBox_path_add_start);
        label_path_add_start_yaw->setObjectName(QString::fromUtf8("label_path_add_start_yaw"));

        horizontalLayout_add_start->addWidget(label_path_add_start_yaw);

        lineEdit_path_add_start_yaw = new QLineEdit(groupBox_path_add_start);
        lineEdit_path_add_start_yaw->setObjectName(QString::fromUtf8("lineEdit_path_add_start_yaw"));

        horizontalLayout_add_start->addWidget(lineEdit_path_add_start_yaw);


        verticalLayout_add->addWidget(groupBox_path_add_start);

        groupBox_path_add_goal = new QGroupBox(groupBox_path_add);
        groupBox_path_add_goal->setObjectName(QString::fromUtf8("groupBox_path_add_goal"));
        horizontalLayout_add_goal = new QHBoxLayout(groupBox_path_add_goal);
        horizontalLayout_add_goal->setObjectName(QString::fromUtf8("horizontalLayout_add_goal"));
        label_path_add_goal_x = new QLabel(groupBox_path_add_goal);
        label_path_add_goal_x->setObjectName(QString::fromUtf8("label_path_add_goal_x"));

        horizontalLayout_add_goal->addWidget(label_path_add_goal_x);

        lineEdit_path_add_goal_x = new QLineEdit(groupBox_path_add_goal);
        lineEdit_path_add_goal_x->setObjectName(QString::fromUtf8("lineEdit_path_add_goal_x"));

        horizontalLayout_add_goal->addWidget(lineEdit_path_add_goal_x);

        label_path_add_goal_y = new QLabel(groupBox_path_add_goal);
        label_path_add_goal_y->setObjectName(QString::fromUtf8("label_path_add_goal_y"));

        horizontalLayout_add_goal->addWidget(label_path_add_goal_y);

        lineEdit_path_add_goal_y = new QLineEdit(groupBox_path_add_goal);
        lineEdit_path_add_goal_y->setObjectName(QString::fromUtf8("lineEdit_path_add_goal_y"));

        horizontalLayout_add_goal->addWidget(lineEdit_path_add_goal_y);

        label_path_add_goal_yaw = new QLabel(groupBox_path_add_goal);
        label_path_add_goal_yaw->setObjectName(QString::fromUtf8("label_path_add_goal_yaw"));

        horizontalLayout_add_goal->addWidget(label_path_add_goal_yaw);

        lineEdit_path_add_goal_yaw = new QLineEdit(groupBox_path_add_goal);
        lineEdit_path_add_goal_yaw->setObjectName(QString::fromUtf8("lineEdit_path_add_goal_yaw"));

        horizontalLayout_add_goal->addWidget(lineEdit_path_add_goal_yaw);


        verticalLayout_add->addWidget(groupBox_path_add_goal);

        groupBox_path_add_planner = new QGroupBox(groupBox_path_add);
        groupBox_path_add_planner->setObjectName(QString::fromUtf8("groupBox_path_add_planner"));
        horizontalLayout_add_planner = new QHBoxLayout(groupBox_path_add_planner);
        horizontalLayout_add_planner->setObjectName(QString::fromUtf8("horizontalLayout_add_planner"));
        label_path_add_planner_global = new QLabel(groupBox_path_add_planner);
        label_path_add_planner_global->setObjectName(QString::fromUtf8("label_path_add_planner_global"));

        horizontalLayout_add_planner->addWidget(label_path_add_planner_global);

        comboBox_path_add_planner_global = new QComboBox(groupBox_path_add_planner);
        comboBox_path_add_planner_global->setObjectName(QString::fromUtf8("comboBox_path_add_planner_global"));

        horizontalLayout_add_planner->addWidget(comboBox_path_add_planner_global);


        verticalLayout_add->addWidget(groupBox_path_add_planner);

        verticalSpacer_path_add_1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_add->addItem(verticalSpacer_path_add_1);

        pushButton_path_add_add = new QPushButton(groupBox_path_add);
        pushButton_path_add_add->setObjectName(QString::fromUtf8("pushButton_path_add_add"));

        verticalLayout_add->addWidget(pushButton_path_add_add);

        verticalSpacer_path_add_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_add->addItem(verticalSpacer_path_add_2);


        gridLayout_pathVisualization->addWidget(groupBox_path_add, 0, 2, 2, 1);

        groupBox_path_files = new QGroupBox(tab_path);
        groupBox_path_files->setObjectName(QString::fromUtf8("groupBox_path_files"));
        verticalLayout_files = new QVBoxLayout(groupBox_path_files);
        verticalLayout_files->setObjectName(QString::fromUtf8("verticalLayout_files"));
        verticalSpacer_path_files_1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_files->addItem(verticalSpacer_path_files_1);

        pushButton_path_files_load = new QPushButton(groupBox_path_files);
        pushButton_path_files_load->setObjectName(QString::fromUtf8("pushButton_path_files_load"));

        verticalLayout_files->addWidget(pushButton_path_files_load);

        verticalSpacer_path_files_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_files->addItem(verticalSpacer_path_files_2);

        pushButton_path_files_save = new QPushButton(groupBox_path_files);
        pushButton_path_files_save->setObjectName(QString::fromUtf8("pushButton_path_files_save"));

        verticalLayout_files->addWidget(pushButton_path_files_save);

        verticalSpacer_path_files_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_files->addItem(verticalSpacer_path_files_3);


        gridLayout_pathVisualization->addWidget(groupBox_path_files, 2, 2, 1, 1);

        tabManager->addTab(tab_path, QString());
        tab_curve = new QWidget();
        tab_curve->setObjectName(QString::fromUtf8("tab_curve"));
        gridLayout = new QGridLayout(tab_curve);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox_curve_curves = new QGroupBox(tab_curve);
        groupBox_curve_curves->setObjectName(QString::fromUtf8("groupBox_curve_curves"));
        sizePolicy.setHeightForWidth(groupBox_curve_curves->sizePolicy().hasHeightForWidth());
        groupBox_curve_curves->setSizePolicy(sizePolicy);
        horizontalLayout_list_2 = new QHBoxLayout(groupBox_curve_curves);
        horizontalLayout_list_2->setObjectName(QString::fromUtf8("horizontalLayout_list_2"));
        tableView_curve_curves = new QTableView(groupBox_curve_curves);
        tableView_curve_curves->setObjectName(QString::fromUtf8("tableView_curve_curves"));
        sizePolicy1.setHeightForWidth(tableView_curve_curves->sizePolicy().hasHeightForWidth());
        tableView_curve_curves->setSizePolicy(sizePolicy1);
        tableView_curve_curves->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        tableView_curve_curves->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        tableView_curve_curves->setEditTriggers(QAbstractItemView::NoEditTriggers);

        horizontalLayout_list_2->addWidget(tableView_curve_curves);


        gridLayout->addWidget(groupBox_curve_curves, 0, 0, 3, 1);

        groupBox_curve_files = new QGroupBox(tab_curve);
        groupBox_curve_files->setObjectName(QString::fromUtf8("groupBox_curve_files"));
        verticalLayout_files_2 = new QVBoxLayout(groupBox_curve_files);
        verticalLayout_files_2->setObjectName(QString::fromUtf8("verticalLayout_files_2"));
        verticalSpacer_curve_files_1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_files_2->addItem(verticalSpacer_curve_files_1);

        pushButton_curve_files_load = new QPushButton(groupBox_curve_files);
        pushButton_curve_files_load->setObjectName(QString::fromUtf8("pushButton_curve_files_load"));

        verticalLayout_files_2->addWidget(pushButton_curve_files_load);

        verticalSpacer_curve_files_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_files_2->addItem(verticalSpacer_curve_files_2);

        pushButton_curve_files_save = new QPushButton(groupBox_curve_files);
        pushButton_curve_files_save->setObjectName(QString::fromUtf8("pushButton_curve_files_save"));

        verticalLayout_files_2->addWidget(pushButton_curve_files_save);

        verticalSpacer_curve_files_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_files_2->addItem(verticalSpacer_curve_files_3);


        gridLayout->addWidget(groupBox_curve_files, 2, 2, 1, 1);

        groupBox_curve_poses = new QGroupBox(tab_curve);
        groupBox_curve_poses->setObjectName(QString::fromUtf8("groupBox_curve_poses"));
        sizePolicy.setHeightForWidth(groupBox_curve_poses->sizePolicy().hasHeightForWidth());
        groupBox_curve_poses->setSizePolicy(sizePolicy);
        horizontalLayout_list_3 = new QHBoxLayout(groupBox_curve_poses);
        horizontalLayout_list_3->setObjectName(QString::fromUtf8("horizontalLayout_list_3"));
        tableView_curve_poses = new QTableView(groupBox_curve_poses);
        tableView_curve_poses->setObjectName(QString::fromUtf8("tableView_curve_poses"));
        sizePolicy1.setHeightForWidth(tableView_curve_poses->sizePolicy().hasHeightForWidth());
        tableView_curve_poses->setSizePolicy(sizePolicy1);
        tableView_curve_poses->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        tableView_curve_poses->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        tableView_curve_poses->setEditTriggers(QAbstractItemView::NoEditTriggers);

        horizontalLayout_list_3->addWidget(tableView_curve_poses);


        gridLayout->addWidget(groupBox_curve_poses, 0, 1, 3, 1);

        groupBox_curve_addPose = new QGroupBox(tab_curve);
        groupBox_curve_addPose->setObjectName(QString::fromUtf8("groupBox_curve_addPose"));
        verticalLayout = new QVBoxLayout(groupBox_curve_addPose);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gridLayout_curve_addPose = new QGridLayout();
        gridLayout_curve_addPose->setObjectName(QString::fromUtf8("gridLayout_curve_addPose"));
        gridLayout_curve_addPose->setContentsMargins(-1, 0, -1, -1);
        label_curve_addPose_x = new QLabel(groupBox_curve_addPose);
        label_curve_addPose_x->setObjectName(QString::fromUtf8("label_curve_addPose_x"));

        gridLayout_curve_addPose->addWidget(label_curve_addPose_x, 0, 0, 1, 1);

        label_curve_addPose_y = new QLabel(groupBox_curve_addPose);
        label_curve_addPose_y->setObjectName(QString::fromUtf8("label_curve_addPose_y"));

        gridLayout_curve_addPose->addWidget(label_curve_addPose_y, 1, 0, 1, 1);

        lineEdit_curve_addPose_yaw = new QLineEdit(groupBox_curve_addPose);
        lineEdit_curve_addPose_yaw->setObjectName(QString::fromUtf8("lineEdit_curve_addPose_yaw"));

        gridLayout_curve_addPose->addWidget(lineEdit_curve_addPose_yaw, 2, 1, 1, 1);

        lineEdit_curve_addPose_x = new QLineEdit(groupBox_curve_addPose);
        lineEdit_curve_addPose_x->setObjectName(QString::fromUtf8("lineEdit_curve_addPose_x"));

        gridLayout_curve_addPose->addWidget(lineEdit_curve_addPose_x, 0, 1, 1, 1);

        label_curve_addPose_yaw = new QLabel(groupBox_curve_addPose);
        label_curve_addPose_yaw->setObjectName(QString::fromUtf8("label_curve_addPose_yaw"));

        gridLayout_curve_addPose->addWidget(label_curve_addPose_yaw, 2, 0, 1, 1);

        lineEdit_curve_addPose_y = new QLineEdit(groupBox_curve_addPose);
        lineEdit_curve_addPose_y->setObjectName(QString::fromUtf8("lineEdit_curve_addPose_y"));

        gridLayout_curve_addPose->addWidget(lineEdit_curve_addPose_y, 1, 1, 1, 1);


        verticalLayout->addLayout(gridLayout_curve_addPose);

        verticalSpacer_curve_addPose_1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_curve_addPose_1);

        pushButton_curve_addPose_add = new QPushButton(groupBox_curve_addPose);
        pushButton_curve_addPose_add->setObjectName(QString::fromUtf8("pushButton_curve_addPose_add"));

        verticalLayout->addWidget(pushButton_curve_addPose_add);

        verticalSpacer_curve_addPose_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_curve_addPose_2);


        gridLayout->addWidget(groupBox_curve_addPose, 1, 2, 1, 1);

        groupBox_curve_addCurve = new QGroupBox(tab_curve);
        groupBox_curve_addCurve->setObjectName(QString::fromUtf8("groupBox_curve_addCurve"));
        groupBox_curve_addCurve->setEnabled(true);
        sizePolicy.setHeightForWidth(groupBox_curve_addCurve->sizePolicy().hasHeightForWidth());
        groupBox_curve_addCurve->setSizePolicy(sizePolicy);
        groupBox_curve_addCurve->setMaximumSize(QSize(16777215, 16777215));
        verticalLayout_add_2 = new QVBoxLayout(groupBox_curve_addCurve);
        verticalLayout_add_2->setObjectName(QString::fromUtf8("verticalLayout_add_2"));
        horizontalLayout_curve_addCurve_type = new QHBoxLayout();
        horizontalLayout_curve_addCurve_type->setObjectName(QString::fromUtf8("horizontalLayout_curve_addCurve_type"));
        horizontalLayout_curve_addCurve_type->setContentsMargins(-1, 0, -1, -1);
        label_curve_addCurve_type = new QLabel(groupBox_curve_addCurve);
        label_curve_addCurve_type->setObjectName(QString::fromUtf8("label_curve_addCurve_type"));

        horizontalLayout_curve_addCurve_type->addWidget(label_curve_addCurve_type);

        comboBox_curve_addCurve_type = new QComboBox(groupBox_curve_addCurve);
        comboBox_curve_addCurve_type->setObjectName(QString::fromUtf8("comboBox_curve_addCurve_type"));

        horizontalLayout_curve_addCurve_type->addWidget(comboBox_curve_addCurve_type);


        verticalLayout_add_2->addLayout(horizontalLayout_curve_addCurve_type);

        verticalSpacer_curve_addCurve_1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_add_2->addItem(verticalSpacer_curve_addCurve_1);

        pushButton_curve_addCurve_add = new QPushButton(groupBox_curve_addCurve);
        pushButton_curve_addCurve_add->setObjectName(QString::fromUtf8("pushButton_curve_addCurve_add"));

        verticalLayout_add_2->addWidget(pushButton_curve_addCurve_add);

        verticalSpacer_curve_addCurve_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_add_2->addItem(verticalSpacer_curve_addCurve_2);


        gridLayout->addWidget(groupBox_curve_addCurve, 0, 2, 1, 1);

        tabManager->addTab(tab_curve, QString());

        verticalLayout_main->addWidget(tabManager);


        retranslateUi(PathVisualPlugin);

        tabManager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(PathVisualPlugin);
    } // setupUi

    void retranslateUi(QWidget *PathVisualPlugin)
    {
        PathVisualPlugin->setWindowTitle(QApplication::translate("PathVisualPlugin", "PathVisualPlugin", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_path_list->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        groupBox_path_list->setTitle(QApplication::translate("PathVisualPlugin", "Path List", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_path_add->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        groupBox_path_add->setTitle(QApplication::translate("PathVisualPlugin", "Add a New Path", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_path_add_start->setToolTip(QApplication::translate("PathVisualPlugin", "The start point of the new path", nullptr));
#endif // QT_NO_TOOLTIP
        groupBox_path_add_start->setTitle(QApplication::translate("PathVisualPlugin", "Start Pose", nullptr));
        label_path_add_start_x->setText(QApplication::translate("PathVisualPlugin", "x", nullptr));
        label_path_add_start_y->setText(QApplication::translate("PathVisualPlugin", "y", nullptr));
        label_path_add_start_yaw->setText(QApplication::translate("PathVisualPlugin", "yaw", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_path_add_goal->setToolTip(QApplication::translate("PathVisualPlugin", "The goal point of the new path", nullptr));
#endif // QT_NO_TOOLTIP
        groupBox_path_add_goal->setTitle(QApplication::translate("PathVisualPlugin", "Goal Pose", nullptr));
        label_path_add_goal_x->setText(QApplication::translate("PathVisualPlugin", "x", nullptr));
        label_path_add_goal_y->setText(QApplication::translate("PathVisualPlugin", "y", nullptr));
        label_path_add_goal_yaw->setText(QApplication::translate("PathVisualPlugin", "yaw", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_path_add_planner->setToolTip(QApplication::translate("PathVisualPlugin", "The planner used to plan the new path", nullptr));
#endif // QT_NO_TOOLTIP
        groupBox_path_add_planner->setTitle(QApplication::translate("PathVisualPlugin", "Planner", nullptr));
#ifndef QT_NO_TOOLTIP
        label_path_add_planner_global->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        label_path_add_planner_global->setText(QApplication::translate("PathVisualPlugin", "Global Planner", nullptr));
#ifndef QT_NO_TOOLTIP
        comboBox_path_add_planner_global->setToolTip(QString());
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        pushButton_path_add_add->setToolTip(QApplication::translate("PathVisualPlugin", "Plan and add the configured path to the path list", nullptr));
#endif // QT_NO_TOOLTIP
        pushButton_path_add_add->setText(QApplication::translate("PathVisualPlugin", "Start Planning", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_path_files->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        groupBox_path_files->setTitle(QApplication::translate("PathVisualPlugin", "For Path Files", nullptr));
#ifndef QT_NO_TOOLTIP
        pushButton_path_files_load->setToolTip(QApplication::translate("PathVisualPlugin", "Load some .json path data files and append them to the path list", nullptr));
#endif // QT_NO_TOOLTIP
        pushButton_path_files_load->setText(QApplication::translate("PathVisualPlugin", "Load Paths", nullptr));
#ifndef QT_NO_TOOLTIP
        pushButton_path_files_save->setToolTip(QApplication::translate("PathVisualPlugin", "Save selected paths data into a .json file", nullptr));
#endif // QT_NO_TOOLTIP
        pushButton_path_files_save->setText(QApplication::translate("PathVisualPlugin", "Save Paths", nullptr));
        tabManager->setTabText(tabManager->indexOf(tab_path), QApplication::translate("PathVisualPlugin", "Path Visualization", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_curve_curves->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        groupBox_curve_curves->setTitle(QApplication::translate("PathVisualPlugin", "Curve List", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_curve_files->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        groupBox_curve_files->setTitle(QApplication::translate("PathVisualPlugin", "For Curve Files", nullptr));
#ifndef QT_NO_TOOLTIP
        pushButton_curve_files_load->setToolTip(QApplication::translate("PathVisualPlugin", "Load some .json path data files and append them to the path list", nullptr));
#endif // QT_NO_TOOLTIP
        pushButton_curve_files_load->setText(QApplication::translate("PathVisualPlugin", "Load Curves", nullptr));
#ifndef QT_NO_TOOLTIP
        pushButton_curve_files_save->setToolTip(QApplication::translate("PathVisualPlugin", "Save selected paths data into a .json file", nullptr));
#endif // QT_NO_TOOLTIP
        pushButton_curve_files_save->setText(QApplication::translate("PathVisualPlugin", "Save Curves", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_curve_poses->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        groupBox_curve_poses->setTitle(QApplication::translate("PathVisualPlugin", "Poses on the Curve", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_curve_addPose->setToolTip(QApplication::translate("PathVisualPlugin", "The start point of the new path", nullptr));
#endif // QT_NO_TOOLTIP
        groupBox_curve_addPose->setTitle(QApplication::translate("PathVisualPlugin", "Add a New Pose", nullptr));
        label_curve_addPose_x->setText(QApplication::translate("PathVisualPlugin", "x", nullptr));
        label_curve_addPose_y->setText(QApplication::translate("PathVisualPlugin", "y", nullptr));
        label_curve_addPose_yaw->setText(QApplication::translate("PathVisualPlugin", "yaw", nullptr));
        pushButton_curve_addPose_add->setText(QApplication::translate("PathVisualPlugin", "Add", nullptr));
#ifndef QT_NO_TOOLTIP
        groupBox_curve_addCurve->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        groupBox_curve_addCurve->setTitle(QApplication::translate("PathVisualPlugin", "Add a New Curve", nullptr));
#ifndef QT_NO_TOOLTIP
        label_curve_addCurve_type->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        label_curve_addCurve_type->setText(QApplication::translate("PathVisualPlugin", "Curve Type", nullptr));
#ifndef QT_NO_TOOLTIP
        comboBox_curve_addCurve_type->setToolTip(QString());
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        pushButton_curve_addCurve_add->setToolTip(QApplication::translate("PathVisualPlugin", "Plan and add the configured path to the path list", nullptr));
#endif // QT_NO_TOOLTIP
        pushButton_curve_addCurve_add->setText(QApplication::translate("PathVisualPlugin", "Add", nullptr));
        tabManager->setTabText(tabManager->indexOf(tab_curve), QApplication::translate("PathVisualPlugin", "Curve Visualization", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PathVisualPlugin: public Ui_PathVisualPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PATH_VISUAL_PLUGIN_H
