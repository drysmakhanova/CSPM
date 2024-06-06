/********************************************************************************
** Form generated from reading UI file 'selector.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SELECTOR_H
#define UI_SELECTOR_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_Selector
{
public:
    QGridLayout *gridLayout;
    QLabel *label_3;
    QPushButton *goJoystick;
    QLabel *label;
    QLabel *label_2;
    QPushButton *goPath;
    QPushButton *goPositions;
    QLabel *label_4;
    QPushButton *sim;

    void setupUi(QDialog *Selector)
    {
        if (Selector->objectName().isEmpty())
            Selector->setObjectName(QString::fromUtf8("Selector"));
        Selector->resize(400, 162);
        gridLayout = new QGridLayout(Selector);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_3 = new QLabel(Selector);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 3, 0, 1, 1);

        goJoystick = new QPushButton(Selector);
        goJoystick->setObjectName(QString::fromUtf8("goJoystick"));

        gridLayout->addWidget(goJoystick, 3, 1, 1, 1);

        label = new QLabel(Selector);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        label_2 = new QLabel(Selector);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        goPath = new QPushButton(Selector);
        goPath->setObjectName(QString::fromUtf8("goPath"));

        gridLayout->addWidget(goPath, 1, 1, 1, 1);

        goPositions = new QPushButton(Selector);
        goPositions->setObjectName(QString::fromUtf8("goPositions"));

        gridLayout->addWidget(goPositions, 0, 1, 1, 1);

        label_4 = new QLabel(Selector);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 2, 0, 1, 1);

        sim = new QPushButton(Selector);
        sim->setObjectName(QString::fromUtf8("sim"));

        gridLayout->addWidget(sim, 2, 1, 1, 1);


        retranslateUi(Selector);

        QMetaObject::connectSlotsByName(Selector);
    } // setupUi

    void retranslateUi(QDialog *Selector)
    {
        Selector->setWindowTitle(QApplication::translate("Selector", "Dialog", nullptr));
        label_3->setText(QApplication::translate("Selector", "Joystick", nullptr));
        goJoystick->setText(QApplication::translate("Selector", "Next", nullptr));
        label->setText(QApplication::translate("Selector", "Enter Positions", nullptr));
        label_2->setText(QApplication::translate("Selector", "Follow path", nullptr));
        goPath->setText(QApplication::translate("Selector", "Next", nullptr));
        goPositions->setText(QApplication::translate("Selector", "Next", nullptr));
        label_4->setText(QApplication::translate("Selector", "Follow path in sim", nullptr));
        sim->setText(QApplication::translate("Selector", "Next", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Selector: public Ui_Selector {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SELECTOR_H
