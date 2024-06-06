/********************************************************************************
** Form generated from reading UI file 'joystickmode.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_JOYSTICKMODE_H
#define UI_JOYSTICKMODE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_JoystickMode
{
public:
    QWidget *centralwidget;
    QLabel *label;
    QLabel *joy_pic;
    QPushButton *pushButton;

    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *JoystickMode)
    {
        if (JoystickMode->objectName().isEmpty())
            JoystickMode->setObjectName(QString::fromUtf8("JoystickMode"));
        JoystickMode->resize(510, 318);
        centralwidget = new QWidget(JoystickMode);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(190, 0, 111, 31));
        joy_pic = new QLabel(centralwidget);
        joy_pic->setObjectName(QString::fromUtf8("joy_pic"));
        joy_pic->setEnabled(true);
        joy_pic->setGeometry(QRect(0, 30, 491, 241));
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(180, 200, 131, 41));
        JoystickMode->setCentralWidget(centralwidget);
        menubar = new QMenuBar(JoystickMode);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 510, 22));
        JoystickMode->setMenuBar(menubar);
        statusbar = new QStatusBar(JoystickMode);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        JoystickMode->setStatusBar(statusbar);

        retranslateUi(JoystickMode);

        QMetaObject::connectSlotsByName(JoystickMode);
    } // setupUi

    void retranslateUi(QMainWindow *JoystickMode)
    {
        JoystickMode->setWindowTitle(QApplication::translate("JoystickMode", "MainWindow", nullptr));
        label->setText(QApplication::translate("JoystickMode", "Joystick mode", nullptr));
        joy_pic->setText(QString());
        pushButton->setText(QApplication::translate("JoystickMode", "Back", nullptr));
    } // retranslateUi

};

namespace Ui {
    class JoystickMode: public Ui_JoystickMode {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_JOYSTICKMODE_H
