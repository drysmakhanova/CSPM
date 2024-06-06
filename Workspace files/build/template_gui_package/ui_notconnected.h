/********************************************************************************
** Form generated from reading UI file 'notconnected.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NOTCONNECTED_H
#define UI_NOTCONNECTED_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_NotConnected
{
public:
    QWidget *centralwidget;
    QLabel *label;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *NotConnected)
    {
        if (NotConnected->objectName().isEmpty())
            NotConnected->setObjectName(QString::fromUtf8("NotConnected"));
        NotConnected->resize(425, 178);
        centralwidget = new QWidget(NotConnected);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(150, 50, 141, 41));
        NotConnected->setCentralWidget(centralwidget);
        menubar = new QMenuBar(NotConnected);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 425, 22));
        NotConnected->setMenuBar(menubar);
        statusbar = new QStatusBar(NotConnected);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        NotConnected->setStatusBar(statusbar);

        retranslateUi(NotConnected);

        QMetaObject::connectSlotsByName(NotConnected);
    } // setupUi

    void retranslateUi(QMainWindow *NotConnected)
    {
        NotConnected->setWindowTitle(QApplication::translate("NotConnected", "MainWindow", nullptr));
        label->setText(QApplication::translate("NotConnected", "Connection error!", nullptr));
    } // retranslateUi

};

namespace Ui {
    class NotConnected: public Ui_NotConnected {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NOTCONNECTED_H
