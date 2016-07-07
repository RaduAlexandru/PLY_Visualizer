#ifndef Visualizer_H
#define Visualizer_H

#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>
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
#include <vtkCamera.h>


#include <math.h>
#include <iterator>

#include <QFileDialog>
#include "Model.h"
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


  vtkSmartPointer<vtkPolyData> wall;
  vtkSmartPointer<vtkPoints> points;
  vtkSmartPointer<vtkCellArray> cells;
  vtkSmartPointer<vtkUnsignedCharArray> colors_original;
  vtkSmartPointer<vtkUnsignedCharArray> colors;


  matrix_type points_wrapped;
  matrix_type points_unwrapped;
  std::vector<double> angles;
  std::vector<double> distances_to_radius;
  std::vector<double> distances_to_center;

  int num_points;
  int point_components;
  double radius;
  double circumference;




  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<InteractorPointPicker> interactor;

public slots:

  virtual void slotExit();
  void on_loadFileButton_clicked();
  void on_clearButton_clicked();
  void on_unwrapButton_clicked();
  void on_colorComboBox_currentIndexChanged(const QString & text);
  void on_perspectiveCheckBox_clicked();


private:

  // Designer form
  Ui_Visualizer *ui;

  bool is_unwrapped=FALSE;

  void clearAll();
  void updateView(int reset_camera=1);
  void getInfo(vtkSmartPointer<vtkPolyData> wall);
  void compute_unwrap();
  double estimateRadius (matrix_type points );
  std::vector<double> computeAngles(matrix_type points);
  std::vector<double>computeDistancesToRadius(matrix_type points, double radius);
  matrix_type vtk_to_vector(vtkSmartPointer<vtkPoints> points);
  vtkSmartPointer<vtkPoints> vector_to_vtk(matrix_type points);
  double interpolate ( double input , double input_start, double input_end, double output_start, double output_end);
  void compute_plain_colors();
  void compute_rgb_colors();
  void compute_depth_colors();
  vtkSmartPointer<vtkUnsignedCharArray> get_colors(vtkSmartPointer<vtkPolyData> wall);

};

#endif
