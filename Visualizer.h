#ifndef Visualizer_H
#define Visualizer_H

#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>
#include <vtkRenderer.h>
#include <QFileDialog>
#include "Model.h"

#include <QMainWindow>

// Forward Qt class declarations
class Ui_Visualizer;

class Visualizer : public QMainWindow
{
  Q_OBJECT
public:

  // Constructor/Destructor
  Visualizer();
  ~Visualizer() {};

  vtkSmartPointer<vtkPolyData> wall;



  vtkSmartPointer<vtkRenderer> renderer;

public slots:

  virtual void slotExit();
  void on_loadFileButton_clicked();

private:

  // Designer form
  Ui_Visualizer *ui;

  void updateView();
};

#endif
