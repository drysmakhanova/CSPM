/********************************************************************************
** Form generated from reading UI file 'trajectory.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TRAJECTORY_H
#define UI_TRAJECTORY_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Trajectory
{
public:
    QWidget *centralwidget;
    QLabel *label;
    QLabel *path_pic;
    QPushButton *pushButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *Trajectory)
    {
        if (Trajectory->objectName().isEmpty())
            Trajectory->setObjectName(QString::fromUtf8("Trajectory"));
        Trajectory->resize(414, 254);
        centralwidget = new QWidget(Trajectory);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(110, 10, 161, 31));
        path_pic = new QLabel(centralwidget);
        path_pic->setObjectName(QString::fromUtf8("path_pic"));
        path_pic->setGeometry(QRect(10, 40, 391, 181));
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(118, 124, 121, 31));
        Trajectory->setCentralWidget(centralwidget);
        menubar = new QMenuBar(Trajectory);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 414, 22));
        Trajectory->setMenuBar(menubar);
        statusbar = new QStatusBar(Trajectory);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        Trajectory->setStatusBar(statusbar);

        retranslateUi(Trajectory);

        QMetaObject::connectSlotsByName(Trajectory);
    } // setupUi

    void retranslateUi(QMainWindow *Trajectory)
    {
        Trajectory->setWindowTitle(QApplication::translate("Trajectory", "MainWindow", nullptr));
        label->setText(QApplication::translate("Trajectory", "Following saved path", nullptr));
        path_pic->setText(QString());
        pushButton->setText(QApplication::translate("Trajectory", "Back", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Trajectory: public Ui_Trajectory {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TRAJECTORY_H
