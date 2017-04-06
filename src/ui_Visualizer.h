/********************************************************************************
** Form generated from reading UI file 'Visualizer.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VISUALIZER_H
#define UI_VISUALIZER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QTextBrowser>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_Visualizer
{
public:
    QAction *actionOpenFile;
    QAction *actionExit;
    QAction *actionPrint;
    QAction *actionHelp;
    QAction *actionSave;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout;
    QVTKWidget *qvtkWidget;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QPushButton *loadFileButton;
    QPushButton *clearButton;
    QPushButton *unwrapButton;
    QComboBox *colorComboBox;
    QTextBrowser *textBrowser;
    QCheckBox *perspectiveCheckBox;
    QPushButton *selectButton;
    QCheckBox *showGridInactiveCheckBox;
    QCheckBox *showGridActiveCheckBox;
    QPushButton *loadFileConfigButton;
    QPushButton *renderToFileButton;
    QLineEdit *numWallsText;
    QLabel *label;
    QLineEdit *aboveThreshText;
    QLineEdit *belowThreshText;

    void setupUi(QMainWindow *Visualizer)
    {
        if (Visualizer->objectName().isEmpty())
            Visualizer->setObjectName(QString::fromUtf8("Visualizer"));
        Visualizer->resize(674, 753);
        actionOpenFile = new QAction(Visualizer);
        actionOpenFile->setObjectName(QString::fromUtf8("actionOpenFile"));
        actionOpenFile->setEnabled(true);
        actionExit = new QAction(Visualizer);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionPrint = new QAction(Visualizer);
        actionPrint->setObjectName(QString::fromUtf8("actionPrint"));
        actionHelp = new QAction(Visualizer);
        actionHelp->setObjectName(QString::fromUtf8("actionHelp"));
        actionSave = new QAction(Visualizer);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        centralwidget = new QWidget(Visualizer);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setSizeConstraint(QLayout::SetFixedSize);
        gridLayout->setHorizontalSpacing(1);
        gridLayout->setVerticalSpacing(0);
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));

        gridLayout->addWidget(qvtkWidget, 0, 0, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        Visualizer->setCentralWidget(centralwidget);
        dockWidget = new QDockWidget(Visualizer);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        dockWidget->setMinimumSize(QSize(200, 36));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        loadFileButton = new QPushButton(dockWidgetContents);
        loadFileButton->setObjectName(QString::fromUtf8("loadFileButton"));
        loadFileButton->setGeometry(QRect(10, 10, 91, 23));
        clearButton = new QPushButton(dockWidgetContents);
        clearButton->setObjectName(QString::fromUtf8("clearButton"));
        clearButton->setGeometry(QRect(10, 360, 75, 23));
        unwrapButton = new QPushButton(dockWidgetContents);
        unwrapButton->setObjectName(QString::fromUtf8("unwrapButton"));
        unwrapButton->setGeometry(QRect(10, 100, 121, 23));
        colorComboBox = new QComboBox(dockWidgetContents);
        colorComboBox->setObjectName(QString::fromUtf8("colorComboBox"));
        colorComboBox->setGeometry(QRect(10, 140, 121, 22));
        textBrowser = new QTextBrowser(dockWidgetContents);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(10, 410, 171, 221));
        perspectiveCheckBox = new QCheckBox(dockWidgetContents);
        perspectiveCheckBox->setObjectName(QString::fromUtf8("perspectiveCheckBox"));
        perspectiveCheckBox->setGeometry(QRect(10, 220, 101, 18));
        perspectiveCheckBox->setChecked(true);
        selectButton = new QPushButton(dockWidgetContents);
        selectButton->setObjectName(QString::fromUtf8("selectButton"));
        selectButton->setGeometry(QRect(10, 320, 91, 23));
        showGridInactiveCheckBox = new QCheckBox(dockWidgetContents);
        showGridInactiveCheckBox->setObjectName(QString::fromUtf8("showGridInactiveCheckBox"));
        showGridInactiveCheckBox->setGeometry(QRect(10, 240, 141, 18));
        showGridActiveCheckBox = new QCheckBox(dockWidgetContents);
        showGridActiveCheckBox->setObjectName(QString::fromUtf8("showGridActiveCheckBox"));
        showGridActiveCheckBox->setGeometry(QRect(10, 260, 141, 20));
        showGridActiveCheckBox->setChecked(true);
        loadFileConfigButton = new QPushButton(dockWidgetContents);
        loadFileConfigButton->setObjectName(QString::fromUtf8("loadFileConfigButton"));
        loadFileConfigButton->setGeometry(QRect(110, 10, 23, 23));
        renderToFileButton = new QPushButton(dockWidgetContents);
        renderToFileButton->setObjectName(QString::fromUtf8("renderToFileButton"));
        renderToFileButton->setGeometry(QRect(10, 180, 121, 23));
        numWallsText = new QLineEdit(dockWidgetContents);
        numWallsText->setObjectName(QString::fromUtf8("numWallsText"));
        numWallsText->setGeometry(QRect(114, 50, 61, 23));
        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 50, 98, 23));
        aboveThreshText = new QLineEdit(dockWidgetContents);
        aboveThreshText->setObjectName(QString::fromUtf8("aboveThreshText"));
        aboveThreshText->setGeometry(QRect(40, 640, 113, 23));
        belowThreshText = new QLineEdit(dockWidgetContents);
        belowThreshText->setObjectName(QString::fromUtf8("belowThreshText"));
        belowThreshText->setGeometry(QRect(40, 670, 113, 23));
        dockWidget->setWidget(dockWidgetContents);
        Visualizer->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget);

        retranslateUi(Visualizer);

        QMetaObject::connectSlotsByName(Visualizer);
    } // setupUi

    void retranslateUi(QMainWindow *Visualizer)
    {
        Visualizer->setWindowTitle(QApplication::translate("Visualizer", "Visualizer", 0, QApplication::UnicodeUTF8));
        actionOpenFile->setText(QApplication::translate("Visualizer", "Open File...", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("Visualizer", "Exit", 0, QApplication::UnicodeUTF8));
        actionPrint->setText(QApplication::translate("Visualizer", "Print", 0, QApplication::UnicodeUTF8));
        actionHelp->setText(QApplication::translate("Visualizer", "Help", 0, QApplication::UnicodeUTF8));
        actionSave->setText(QApplication::translate("Visualizer", "Save", 0, QApplication::UnicodeUTF8));
        loadFileButton->setText(QApplication::translate("Visualizer", "Load Data", 0, QApplication::UnicodeUTF8));
        clearButton->setText(QApplication::translate("Visualizer", "Clear View", 0, QApplication::UnicodeUTF8));
        unwrapButton->setText(QApplication::translate("Visualizer", "Wrap/ Unwrap", 0, QApplication::UnicodeUTF8));
        textBrowser->setHtml(QApplication::translate("Visualizer", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">right click 	-select cell</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">f 	- zoom to point</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">w	- wireframe mode</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">s	- su"
                        "rface mode</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">r	- reset camera</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">3 	- stereo mode</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">shift+drag    	- move</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">ctrl+drag 	- tilt</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">scroll 	- zoom</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin"
                        "-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">p 	-pick</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"><br /></p></body></html>", 0, QApplication::UnicodeUTF8));
        perspectiveCheckBox->setText(QApplication::translate("Visualizer", "Perspective", 0, QApplication::UnicodeUTF8));
        selectButton->setText(QApplication::translate("Visualizer", "Select defects", 0, QApplication::UnicodeUTF8));
        showGridInactiveCheckBox->setText(QApplication::translate("Visualizer", "ShowGridInactive", 0, QApplication::UnicodeUTF8));
        showGridActiveCheckBox->setText(QApplication::translate("Visualizer", "ShowGridActive", 0, QApplication::UnicodeUTF8));
        loadFileConfigButton->setText(QString());
        renderToFileButton->setText(QApplication::translate("Visualizer", "Render to File", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Visualizer", "Num. of walls", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Visualizer: public Ui_Visualizer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VISUALIZER_H
