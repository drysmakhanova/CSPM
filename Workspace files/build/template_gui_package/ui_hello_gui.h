/********************************************************************************
** Form generated from reading UI file 'hello_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HELLO_GUI_H
#define UI_HELLO_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HelloGui
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpinBox *alpha;
    QLabel *label_2;
    QSpinBox *beta;
    QLabel *label_3;
    QSpinBox *gamma;
    QLabel *chatter_lbl;
    QLabel *chatter;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *hi_button;
        QPushButton *pushButton2;

    void setupUi(QWidget *HelloGui)
    {
        if (HelloGui->objectName().isEmpty())
            HelloGui->setObjectName(QString::fromUtf8("HelloGui"));
        HelloGui->resize(1089, 426);
        verticalLayout = new QVBoxLayout(HelloGui);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(HelloGui);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        alpha = new QSpinBox(HelloGui);
        alpha->setObjectName(QString::fromUtf8("alpha"));
        alpha->setMaximum(1000);

        horizontalLayout->addWidget(alpha);

        label_2 = new QLabel(HelloGui);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        beta = new QSpinBox(HelloGui);
        beta->setObjectName(QString::fromUtf8("beta"));
        beta->setMaximum(1000);

        horizontalLayout->addWidget(beta);

        label_3 = new QLabel(HelloGui);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout->addWidget(label_3);

        gamma = new QSpinBox(HelloGui);
        gamma->setObjectName(QString::fromUtf8("gamma"));
        gamma->setMaximum(1000);

        horizontalLayout->addWidget(gamma);

        chatter_lbl = new QLabel(HelloGui);
        chatter_lbl->setObjectName(QString::fromUtf8("chatter_lbl"));

        horizontalLayout->addWidget(chatter_lbl);

        chatter = new QLabel(HelloGui);
        chatter->setObjectName(QString::fromUtf8("chatter"));

        horizontalLayout->addWidget(chatter);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));

        hi_button = new QPushButton(HelloGui);
        hi_button->setObjectName(QString::fromUtf8("hi_button"));

        horizontalLayout_2->addWidget(hi_button);
	        pushButton2 = new QPushButton(HelloGui);
        pushButton2->setObjectName(QString::fromUtf8("pushButton2"));

        horizontalLayout_2->addWidget(pushButton2);
        verticalLayout->addLayout(horizontalLayout_2);


        retranslateUi(HelloGui);

        QMetaObject::connectSlotsByName(HelloGui);
    } // setupUi

    void retranslateUi(QWidget *HelloGui)
    {
        HelloGui->setWindowTitle(QApplication::translate("HelloGui", "Form", nullptr));
        label->setText(QApplication::translate("HelloGui", "                      Theta1:", nullptr));
        label_2->setText(QApplication::translate("HelloGui", "                               Theta2:", nullptr));
        label_3->setText(QApplication::translate("HelloGui", "                            Theta3:", nullptr));
        chatter_lbl->setText(QString());
        chatter->setText(QString());
        hi_button->setText(QApplication::translate("HelloGui", "Submit", nullptr));
        pushButton2->setText(QApplication::translate("HelloGui", "Back", nullptr));
    } // retranslateUi

};

namespace Ui {
    class HelloGui: public Ui_HelloGui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HELLO_GUI_H
