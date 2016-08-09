#ifndef Visualizer_H
#define Visualizer_H

#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkRenderer.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>
#include <vtkBox.h>
#include <vtkSphere.h>
#include <vtkClipPolyData.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkPointPicker.h>
#include <vtkCellPicker.h>
#include <vtkPropPicker.h>
#include <vtkCamera.h>
#include <vtkDelaunay2D.h>
#include <vtkPolyDataNormals.h>
#include <vtkOBJImporter.h>


#include <math.h>
#include <iterator>
#include <boost/algorithm/string/predicate.hpp>


#include <QFileDialog>
#include "Model.h"
#include "Utils.h"
#include "InteractorPointPicker.h"

#include <QMainWindow>

typedef std::vector<double> row_type;
typedef std::vector<row_type> matrix_type;

// Forward Qt class declarations
class Ui_Visualizer;
//class Model;

class Visualizer : public QMainWindow
{
  Q_OBJECT
public:

  // Constructor/Destructor
  Visualizer();
  ~Visualizer() {};

  std::shared_ptr<Model> model;
  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<InteractorPointPicker> interactor;

  

public slots:

  virtual void slotExit();
  void on_loadFileButton_clicked();
  void on_clearButton_clicked();
  void on_unwrapButton_clicked();
  void on_colorComboBox_currentIndexChanged(const QString & text);
  void on_perspectiveCheckBox_clicked();
  void on_selectButton_clicked();
  void on_gridButton_clicked();


private:

  // Designer form
  Ui_Visualizer *ui;

  void clearAll();
  void updateView(int reset_camera=1);


};

#endif
