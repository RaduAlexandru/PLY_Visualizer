/********************************************************************************
** Form generated from reading UI file 'ConfigDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONFIGDIALOG_H
#define UI_CONFIGDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ConfigDialog
{
public:
    QPushButton *pushButton;
    QGroupBox *groupBox_2;
    QWidget *layoutWidget;
    QGridLayout *gridLayout_3;
    QLabel *label;
    QLineEdit *numWallsText;
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
            ConfigDialog->setObjectName(QStringLiteral("ConfigDialog"));
        ConfigDialog->resize(267, 475);
        pushButton = new QPushButton(ConfigDialog);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(170, 430, 80, 23));
        groupBox_2 = new QGroupBox(ConfigDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 90, 231, 131));
        layoutWidget = new QWidget(groupBox_2);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 30, 206, 81));
        gridLayout_3 = new QGridLayout(layoutWidget);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout_3->addWidget(label, 0, 0, 1, 1);

        numWallsText = new QLineEdit(layoutWidget);
        numWallsText->setObjectName(QStringLiteral("numWallsText"));

        gridLayout_3->addWidget(numWallsText, 0, 1, 1, 1);

        clearUnwrapButton = new QPushButton(layoutWidget);
        clearUnwrapButton->setObjectName(QStringLiteral("clearUnwrapButton"));

        gridLayout_3->addWidget(clearUnwrapButton, 3, 0, 1, 1);

        deformWallscheckBox = new QCheckBox(layoutWidget);
        deformWallscheckBox->setObjectName(QStringLiteral("deformWallscheckBox"));
        deformWallscheckBox->setLayoutDirection(Qt::LeftToRight);
        deformWallscheckBox->setChecked(true);

        gridLayout_3->addWidget(deformWallscheckBox, 2, 0, 1, 2);

        groupBox_3 = new QGroupBox(ConfigDialog);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(20, 230, 231, 181));
        layoutWidget1 = new QWidget(groupBox_3);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 30, 211, 141));
        gridLayout_4 = new QGridLayout(layoutWidget1);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        pathlabel = new QLabel(layoutWidget1);
        pathlabel->setObjectName(QStringLiteral("pathlabel"));
        pathlabel->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pathlabel->sizePolicy().hasHeightForWidth());
        pathlabel->setSizePolicy(sizePolicy);

        gridLayout_4->addWidget(pathlabel, 0, 0, 1, 1);

        renderFullImgcheckBox = new QCheckBox(layoutWidget1);
        renderFullImgcheckBox->setObjectName(QStringLiteral("renderFullImgcheckBox"));
        renderFullImgcheckBox->setChecked(true);

        gridLayout_4->addWidget(renderFullImgcheckBox, 1, 0, 1, 2);

        fullImgMagText = new QLineEdit(layoutWidget1);
        fullImgMagText->setObjectName(QStringLiteral("fullImgMagText"));

        gridLayout_4->addWidget(fullImgMagText, 1, 2, 1, 1);

        renderGridUnwrappedcheckBox = new QCheckBox(layoutWidget1);
        renderGridUnwrappedcheckBox->setObjectName(QStringLiteral("renderGridUnwrappedcheckBox"));
        renderGridUnwrappedcheckBox->setChecked(true);

        gridLayout_4->addWidget(renderGridUnwrappedcheckBox, 2, 0, 1, 2);

        gridUnwrappedMagText = new QLineEdit(layoutWidget1);
        gridUnwrappedMagText->setObjectName(QStringLiteral("gridUnwrappedMagText"));

        gridLayout_4->addWidget(gridUnwrappedMagText, 2, 2, 1, 1);

        renderGridWrappedcheckBox = new QCheckBox(layoutWidget1);
        renderGridWrappedcheckBox->setObjectName(QStringLiteral("renderGridWrappedcheckBox"));
        renderGridWrappedcheckBox->setChecked(true);

        gridLayout_4->addWidget(renderGridWrappedcheckBox, 3, 0, 1, 2);

        gridWrappedMagText = new QLineEdit(layoutWidget1);
        gridWrappedMagText->setObjectName(QStringLiteral("gridWrappedMagText"));

        gridLayout_4->addWidget(gridWrappedMagText, 3, 2, 1, 1);

        renderWallscheckBox = new QCheckBox(layoutWidget1);
        renderWallscheckBox->setObjectName(QStringLiteral("renderWallscheckBox"));
        renderWallscheckBox->setChecked(true);

        gridLayout_4->addWidget(renderWallscheckBox, 4, 0, 1, 2);

        wallsMagText = new QLineEdit(layoutWidget1);
        wallsMagText->setObjectName(QStringLiteral("wallsMagText"));

        gridLayout_4->addWidget(wallsMagText, 4, 2, 1, 1);

        pathText = new QLineEdit(layoutWidget1);
        pathText->setObjectName(QStringLiteral("pathText"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pathText->sizePolicy().hasHeightForWidth());
        pathText->setSizePolicy(sizePolicy1);

        gridLayout_4->addWidget(pathText, 0, 1, 1, 2);

        groupBox = new QGroupBox(ConfigDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(20, 20, 221, 61));
        experimentalLoadingcheckBox = new QCheckBox(groupBox);
        experimentalLoadingcheckBox->setObjectName(QStringLiteral("experimentalLoadingcheckBox"));
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
        ConfigDialog->setWindowTitle(QApplication::translate("ConfigDialog", "Dialog", 0));
        pushButton->setText(QApplication::translate("ConfigDialog", "Accept", 0));
        groupBox_2->setTitle(QApplication::translate("ConfigDialog", "Unwrapping", 0));
        label->setText(QApplication::translate("ConfigDialog", "Number of walls", 0));
        clearUnwrapButton->setText(QApplication::translate("ConfigDialog", "Clear unwrap", 0));
#ifndef QT_NO_TOOLTIP
        deformWallscheckBox->setToolTip(QApplication::translate("ConfigDialog", "Deform the walls slightly so that the transition between them is smoother when unwrapping. Adds computation time to the unwrapping", 0));
#endif // QT_NO_TOOLTIP
        deformWallscheckBox->setText(QApplication::translate("ConfigDialog", "Deform unwrapped walls", 0));
        groupBox_3->setTitle(QApplication::translate("ConfigDialog", "Saving to file", 0));
        pathlabel->setText(QApplication::translate("ConfigDialog", "Path:", 0));
        renderFullImgcheckBox->setText(QApplication::translate("ConfigDialog", "Render full image", 0));
        renderGridUnwrappedcheckBox->setText(QApplication::translate("ConfigDialog", "Render grid unwrapped", 0));
        renderGridWrappedcheckBox->setText(QApplication::translate("ConfigDialog", "Render grid wrapped", 0));
        renderWallscheckBox->setText(QApplication::translate("ConfigDialog", "Render walls", 0));
#ifndef QT_NO_TOOLTIP
        groupBox->setToolTip(QApplication::translate("ConfigDialog", "Can lead do great increase in the processing of some dataset. However it is prone to error in the case of some obj files.", 0));
#endif // QT_NO_TOOLTIP
        groupBox->setTitle(QApplication::translate("ConfigDialog", "Data Loading", 0));
#ifndef QT_NO_TOOLTIP
        experimentalLoadingcheckBox->setToolTip(QApplication::translate("ConfigDialog", "Can lead to great increase in performance in the case of some obj files.", 0));
#endif // QT_NO_TOOLTIP
        experimentalLoadingcheckBox->setText(QApplication::translate("ConfigDialog", "Use fast OBJ reader", 0));
    } // retranslateUi

};

namespace Ui {
    class ConfigDialog: public Ui_ConfigDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONFIGDIALOG_H
