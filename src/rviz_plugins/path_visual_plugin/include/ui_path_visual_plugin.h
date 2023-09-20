/********************************************************************************
** Form generated from reading UI file 'path_visual_plugin.ui'
**
** Created by: Qt User Interface Compiler version 5.9.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PATH_VISUAL_PLUGIN_H
#define UI_PATH_VISUAL_PLUGIN_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
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
    QVBoxLayout *verticalLayout_5;
    QTabWidget *tab_manager;
    QWidget *tab_path_visualization;
    QGridLayout *gridLayout_4;
    QGroupBox *groupBox_3;
    QHBoxLayout *horizontalLayout_6;
    QTableView *tableView;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox_5;
    QHBoxLayout *horizontalLayout;
    QLabel *label_4;
    QLineEdit *lineEdit;
    QLabel *label_5;
    QLineEdit *lineEdit_2;
    QGroupBox *groupBox_6;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_6;
    QLineEdit *lineEdit_3;
    QLabel *label_7;
    QLineEdit *lineEdit_4;
    QGroupBox *groupBox_7;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_8;
    QComboBox *comboBox;
    QPushButton *pushButton;
    QSpacerItem *verticalSpacer_2;
    QGroupBox *groupBox_10;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *pushButton_4;
    QGroupBox *groupBox_8;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;

    void setupUi(QWidget *PathVisualPlugin)
    {
        if (PathVisualPlugin->objectName().isEmpty())
            PathVisualPlugin->setObjectName(QStringLiteral("PathVisualPlugin"));
        PathVisualPlugin->resize(800, 600);
        verticalLayout_5 = new QVBoxLayout(PathVisualPlugin);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        tab_manager = new QTabWidget(PathVisualPlugin);
        tab_manager->setObjectName(QStringLiteral("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_path_visualization = new QWidget();
        tab_path_visualization->setObjectName(QStringLiteral("tab_path_visualization"));
        gridLayout_4 = new QGridLayout(tab_path_visualization);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        groupBox_3 = new QGroupBox(tab_path_visualization);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        horizontalLayout_6 = new QHBoxLayout(groupBox_3);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        tableView = new QTableView(groupBox_3);
        tableView->setObjectName(QStringLiteral("tableView"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(tableView->sizePolicy().hasHeightForWidth());
        tableView->setSizePolicy(sizePolicy);

        horizontalLayout_6->addWidget(tableView);


        gridLayout_4->addWidget(groupBox_3, 0, 0, 1, 2);

        groupBox_4 = new QGroupBox(tab_path_visualization);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(groupBox_4->sizePolicy().hasHeightForWidth());
        groupBox_4->setSizePolicy(sizePolicy1);
        verticalLayout = new QVBoxLayout(groupBox_4);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        groupBox_5 = new QGroupBox(groupBox_4);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        horizontalLayout = new QHBoxLayout(groupBox_5);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_4 = new QLabel(groupBox_5);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout->addWidget(label_4);

        lineEdit = new QLineEdit(groupBox_5);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));

        horizontalLayout->addWidget(lineEdit);

        label_5 = new QLabel(groupBox_5);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout->addWidget(label_5);

        lineEdit_2 = new QLineEdit(groupBox_5);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));

        horizontalLayout->addWidget(lineEdit_2);


        verticalLayout->addWidget(groupBox_5);

        groupBox_6 = new QGroupBox(groupBox_4);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        horizontalLayout_2 = new QHBoxLayout(groupBox_6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_6 = new QLabel(groupBox_6);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_2->addWidget(label_6);

        lineEdit_3 = new QLineEdit(groupBox_6);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));

        horizontalLayout_2->addWidget(lineEdit_3);

        label_7 = new QLabel(groupBox_6);
        label_7->setObjectName(QStringLiteral("label_7"));

        horizontalLayout_2->addWidget(label_7);

        lineEdit_4 = new QLineEdit(groupBox_6);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));

        horizontalLayout_2->addWidget(lineEdit_4);


        verticalLayout->addWidget(groupBox_6);

        groupBox_7 = new QGroupBox(groupBox_4);
        groupBox_7->setObjectName(QStringLiteral("groupBox_7"));
        horizontalLayout_3 = new QHBoxLayout(groupBox_7);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label_8 = new QLabel(groupBox_7);
        label_8->setObjectName(QStringLiteral("label_8"));

        horizontalLayout_3->addWidget(label_8);

        comboBox = new QComboBox(groupBox_7);
        comboBox->setObjectName(QStringLiteral("comboBox"));

        horizontalLayout_3->addWidget(comboBox);


        verticalLayout->addWidget(groupBox_7);

        pushButton = new QPushButton(groupBox_4);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        verticalLayout->addWidget(pushButton);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);


        gridLayout_4->addWidget(groupBox_4, 0, 2, 2, 1);

        groupBox_10 = new QGroupBox(tab_path_visualization);
        groupBox_10->setObjectName(QStringLiteral("groupBox_10"));
        horizontalLayout_5 = new QHBoxLayout(groupBox_10);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        pushButton_4 = new QPushButton(groupBox_10);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));

        horizontalLayout_5->addWidget(pushButton_4);


        gridLayout_4->addWidget(groupBox_10, 1, 0, 1, 1);

        groupBox_8 = new QGroupBox(tab_path_visualization);
        groupBox_8->setObjectName(QStringLiteral("groupBox_8"));
        horizontalLayout_4 = new QHBoxLayout(groupBox_8);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        pushButton_2 = new QPushButton(groupBox_8);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        horizontalLayout_4->addWidget(pushButton_2);

        pushButton_3 = new QPushButton(groupBox_8);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));

        horizontalLayout_4->addWidget(pushButton_3);


        gridLayout_4->addWidget(groupBox_8, 1, 1, 1, 1);

        tab_manager->addTab(tab_path_visualization, QString());

        verticalLayout_5->addWidget(tab_manager);


        retranslateUi(PathVisualPlugin);

        tab_manager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(PathVisualPlugin);
    } // setupUi

    void retranslateUi(QWidget *PathVisualPlugin)
    {
        PathVisualPlugin->setWindowTitle(QApplication::translate("PathVisualPlugin", "PathVisualPlugin", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("PathVisualPlugin", "Path List", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("PathVisualPlugin", "Add a New Path", Q_NULLPTR));
        groupBox_5->setTitle(QApplication::translate("PathVisualPlugin", "Start Point", Q_NULLPTR));
        label_4->setText(QApplication::translate("PathVisualPlugin", "x", Q_NULLPTR));
        label_5->setText(QApplication::translate("PathVisualPlugin", "y", Q_NULLPTR));
        groupBox_6->setTitle(QApplication::translate("PathVisualPlugin", "Goal Point", Q_NULLPTR));
        label_6->setText(QApplication::translate("PathVisualPlugin", "x", Q_NULLPTR));
        label_7->setText(QApplication::translate("PathVisualPlugin", "y", Q_NULLPTR));
        groupBox_7->setTitle(QApplication::translate("PathVisualPlugin", "Planner", Q_NULLPTR));
        label_8->setText(QApplication::translate("PathVisualPlugin", "Global Planner", Q_NULLPTR));
        pushButton->setText(QApplication::translate("PathVisualPlugin", "Add Path", Q_NULLPTR));
        groupBox_10->setTitle(QApplication::translate("PathVisualPlugin", "Remove Selected Path", Q_NULLPTR));
        pushButton_4->setText(QApplication::translate("PathVisualPlugin", "Remove Path", Q_NULLPTR));
        groupBox_8->setTitle(QApplication::translate("PathVisualPlugin", "Save or Load the Path Files", Q_NULLPTR));
        pushButton_2->setText(QApplication::translate("PathVisualPlugin", "Save Paths", Q_NULLPTR));
        pushButton_3->setText(QApplication::translate("PathVisualPlugin", "Load Paths", Q_NULLPTR));
        tab_manager->setTabText(tab_manager->indexOf(tab_path_visualization), QApplication::translate("PathVisualPlugin", "Path Visualization", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PathVisualPlugin: public Ui_PathVisualPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PATH_VISUAL_PLUGIN_H
