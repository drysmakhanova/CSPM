/********************************************************************************
** Form generated from reading UI file 'connect.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONNECT_H
#define UI_CONNECT_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Connect
{
public:
    QWidget *centralwidget;
    QPushButton *pushButton;
    QLabel *label;
    QLabel *connect_pic;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *Connect)
    {
        if (Connect->objectName().isEmpty())
            Connect->setObjectName(QString::fromUtf8("Connect"));
        Connect->resize(541, 336);
        centralwidget = new QWidget(Connect);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(200, 210, 89, 25));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(170, 20, 181, 31));
        connect_pic = new QLabel(centralwidget);
        connect_pic->setObjectName(QString::fromUtf8("connect_pic"));
        connect_pic->setGeometry(QRect(160, 60, 271, 131));
        Connect->setCentralWidget(centralwidget);
        menubar = new QMenuBar(Connect);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 541, 22));
        Connect->setMenuBar(menubar);
        statusbar = new QStatusBar(Connect);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        Connect->setStatusBar(statusbar);

        retranslateUi(Connect);

        QMetaObject::connectSlotsByName(Connect);
    } // setupUi

    void retranslateUi(QMainWindow *Connect)
    {
        Connect->setWindowTitle(QApplication::translate("Connect", "MainWindow", nullptr));
        pushButton->setText(QApplication::translate("Connect", "Continue", nullptr));
        label->setText(QApplication::translate("Connect", "Connection established", nullptr));
        connect_pic->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class Connect: public Ui_Connect {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONNECT_H
