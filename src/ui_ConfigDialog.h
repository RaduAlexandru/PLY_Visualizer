/********************************************************************************
** Form generated from reading UI file 'ConfigDialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONFIGDIALOG_H
#define UI_CONFIGDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ConfigDialog
{
public:
    QPushButton *pushButton;
    QGroupBox *groupBox_2;
    QWidget *layoutWidget;
    QGridLayout *gridLayout_3;
    QPushButton *clearUnwrapButton;
    QCheckBox *deformWallscheckBox;
    QGroupBox *groupBox_3;
    QWidget *layoutWidget1;
    QGridLayout *gridLayout_4;
    QLabel *pathlabel;
    QCheckBox *renderFullImgcheckBox;
    QLineEdit *fullImgMagText;
    QCheckBox *renderGridUnwrappedcheckBox;
    QLineEdit *gridUnwrappedMagText;
    QCheckBox *renderGridWrappedcheckBox;
    QLineEdit *gridWrappedMagText;
    QCheckBox *renderWallscheckBox;
    QLineEdit *wallsMagText;
    QLineEdit *pathText;
    QGroupBox *groupBox;
    QCheckBox *experimentalLoadingcheckBox;

    void setupUi(QDialog *ConfigDialog)
    {
        if (ConfigDialog->objectName().isEmpty())
            ConfigDialog->setObjectName(QString::fromUtf8("ConfigDialog"));
        ConfigDialog->resize(267, 459);
        pushButton = new QPushButton(ConfigDialog);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(170, 410, 80, 23));
        groupBox_2 = new QGroupBox(ConfigDialog);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 90, 231, 101));
        layoutWidget = new QWidget(groupBox_2);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 30, 206, 61));
        gridLayout_3 = new QGridLayout(layoutWidget);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        clearUnwrapButton = new QPushButton(layoutWidget);
        clearUnwrapButton->setObjectName(QString::fromUtf8("clearUnwrapButton"));

        gridLayout_3->addWidget(clearUnwrapButton, 2, 0, 1, 1);

        deformWallscheckBox = new QCheckBox(layoutWidget);
        deformWallscheckBox->setObjectName(QString::fromUtf8("deformWallscheckBox"));
        deformWallscheckBox->setLayoutDirection(Qt::LeftToRight);
        deformWallscheckBox->setChecked(false);

        gridLayout_3->addWidget(deformWallscheckBox, 1, 0, 1, 2);

        groupBox_3 = new QGroupBox(ConfigDialog);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(20, 210, 231, 181));
        layoutWidget1 = new QWidget(groupBox_3);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 30, 211, 141));
        gridLayout_4 = new QGridLayout(layoutWidget1);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        pathlabel = new QLabel(layoutWidget1);
        pathlabel->setObjectName(QString::fromUtf8("pathlabel"));
        pathlabel->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pathlabel->sizePolicy().hasHeightForWidth());
        pathlabel->setSizePolicy(sizePolicy);

        gridLayout_4->addWidget(pathlabel, 0, 0, 1, 1);

        renderFullImgcheckBox = new QCheckBox(layoutWidget1);
        renderFullImgcheckBox->setObjectName(QString::fromUtf8("renderFullImgcheckBox"));
        renderFullImgcheckBox->setChecked(true);

        gridLayout_4->addWidget(renderFullImgcheckBox, 1, 0, 1, 2);

        fullImgMagText = new QLineEdit(layoutWidget1);
        fullImgMagText->setObjectName(QString::fromUtf8("fullImgMagText"));

        gridLayout_4->addWidget(fullImgMagText, 1, 2, 1, 1);

        renderGridUnwrappedcheckBox = new QCheckBox(layoutWidget1);
        renderGridUnwrappedcheckBox->setObjectName(QString::fromUtf8("renderGridUnwrappedcheckBox"));
        renderGridUnwrappedcheckBox->setChecked(true);

        gridLayout_4->addWidget(renderGridUnwrappedcheckBox, 2, 0, 1, 2);

        gridUnwrappedMagText = new QLineEdit(layoutWidget1);
        gridUnwrappedMagText->setObjectName(QString::fromUtf8("gridUnwrappedMagText"));

        gridLayout_4->addWidget(gridUnwrappedMagText, 2, 2, 1, 1);

        renderGridWrappedcheckBox = new QCheckBox(layoutWidget1);
        renderGridWrappedcheckBox->setObjectName(QString::fromUtf8("renderGridWrappedcheckBox"));
        renderGridWrappedcheckBox->setChecked(true);

        gridLayout_4->addWidget(renderGridWrappedcheckBox, 3, 0, 1, 2);

        gridWrappedMagText = new QLineEdit(layoutWidget1);
        gridWrappedMagText->setObjectName(QString::fromUtf8("gridWrappedMagText"));

        gridLayout_4->addWidget(gridWrappedMagText, 3, 2, 1, 1);

        renderWallscheckBox = new QCheckBox(layoutWidget1);
        renderWallscheckBox->setObjectName(QString::fromUtf8("renderWallscheckBox"));
        renderWallscheckBox->setChecked(true);

        gridLayout_4->addWidget(renderWallscheckBox, 4, 0, 1, 2);

        wallsMagText = new QLineEdit(layoutWidget1);
        wallsMagText->setObjectName(QString::fromUtf8("wallsMagText"));

        gridLayout_4->addWidget(wallsMagText, 4, 2, 1, 1);

        pathText = new QLineEdit(layoutWidget1);
        pathText->setObjectName(QString::fromUtf8("pathText"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pathText->sizePolicy().hasHeightForWidth());
        pathText->setSizePolicy(sizePolicy1);

        gridLayout_4->addWidget(pathText, 0, 1, 1, 2);

        groupBox = new QGroupBox(ConfigDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(20, 20, 231, 61));
        experimentalLoadingcheckBox = new QCheckBox(groupBox);
        experimentalLoadingcheckBox->setObjectName(QString::fromUtf8("experimentalLoadingcheckBox"));
        experimentalLoadingcheckBox->setGeometry(QRect(10, 30, 181, 21));
        experimentalLoadingcheckBox->setLayoutDirection(Qt::LeftToRight);
        experimentalLoadingcheckBox->setChecked(true);
        experimentalLoadingcheckBox->setTristate(false);

        retranslateUi(ConfigDialog);
        QObject::connect(pushButton, SIGNAL(clicked()), ConfigDialog, SLOT(accept()));

        QMetaObject::connectSlotsByName(ConfigDialog);
    } // setupUi

    void retranslateUi(QDialog *ConfigDialog)
    {
        ConfigDialog->setWindowTitle(QApplication::translate("ConfigDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("ConfigDialog", "Accept", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("ConfigDialog", "Unwrapping", 0, QApplication::UnicodeUTF8));
        clearUnwrapButton->setText(QApplication::translate("ConfigDialog", "Clear unwrap", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        deformWallscheckBox->setToolTip(QApplication::translate("ConfigDialog", "Deform the walls slightly so that the transition between them is smoother when unwrapping. Adds computation time to the unwrapping", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        deformWallscheckBox->setText(QApplication::translate("ConfigDialog", "Deform unwrapped walls", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("ConfigDialog", "Saving to file", 0, QApplication::UnicodeUTF8));
        pathlabel->setText(QApplication::translate("ConfigDialog", "Path:", 0, QApplication::UnicodeUTF8));
        renderFullImgcheckBox->setText(QApplication::translate("ConfigDialog", "Render full image", 0, QApplication::UnicodeUTF8));
        renderGridUnwrappedcheckBox->setText(QApplication::translate("ConfigDialog", "Render grid unwrapped", 0, QApplication::UnicodeUTF8));
        renderGridWrappedcheckBox->setText(QApplication::translate("ConfigDialog", "Render grid wrapped", 0, QApplication::UnicodeUTF8));
        renderWallscheckBox->setText(QApplication::translate("ConfigDialog", "Render walls", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        groupBox->setToolTip(QApplication::translate("ConfigDialog", "Can lead do great increase in the processing of some dataset. However it is prone to error in the case of some obj files.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        groupBox->setTitle(QApplication::translate("ConfigDialog", "Data Loading", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        experimentalLoadingcheckBox->setToolTip(QApplication::translate("ConfigDialog", "Can lead to great increase in performance in the case of some obj files.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        experimentalLoadingcheckBox->setText(QApplication::translate("ConfigDialog", "Use fast OBJ reader", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ConfigDialog: public Ui_ConfigDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONFIGDIALOG_H
