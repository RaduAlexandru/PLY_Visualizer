#ifndef CONFIGDIALOG_H
#define CONFIGDIALOG_H

#include <QDialog>
#include "ui_ConfigDialog.h"

class Ui_ConfigDialog;


class ConfigDialog : public QDialog, public Ui::ConfigDialog{
    Q_OBJECT

public:
    ConfigDialog( QWidget * parent = 0);


private:
    // Ui::ConfigDialog ui;
};


#endif


//
//
//
// #include <ui_mainwindow.h>
//
// class MyClass
// {
// public:
//     MyClass(Ui::MainWindow *ui);
//
// private:
//     Ui::MainWindow *myUi;
// };


// #include <ui_Visualizer.h>
//
// class ConfigDialog
// {
// public:
//     ConfigDialog(Ui::Visualizer *ui);
//
// private:
//     Ui::Visualizer *myUi;
// };

// #ifndef CONFIGDIALOG_H
// #define CONFIGDIALOG_H
//
// // #include <QtGui>
// #include <QDialog>
//
//
// class ConfigDialog : public QDialog
// {
//   Q_OBJECT
//   public:
//     ConfigDialog(QWidget *parent = 0);
//
// };
//
// #endif
