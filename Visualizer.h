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
#include <vtkPlaneSource.h>
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
#include <vtkMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkSphereSource.h>
#include <vtkPNGReader.h>
#include <vtkImageData.h>
#include <vtkAssembly.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkTextureUnitManager.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkLine.h>

#include "vtkTexturingHelper.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



#include <math.h>
#include <iterator>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


#include <QFileDialog>
#include "Model.h"
#include "Utils.h"
#include "OBJReader2.h"
#include "InteractorPointPicker.h"

#include <QMainWindow>

typedef std::vector<double> row_type;
typedef std::vector<row_type> matrix_type;

struct plane_struct {
    pcl::ModelCoefficients coef;
    double angle;
};

struct line_struct {
    row_type direction;
    row_type point;
    double angle;   //angle of the line with respect to the x axis
    double distance; //distance from a point to this line. Will be changed in each iteration when a point is checked
    int index; //index assigned to indicat the ordering with respect to the angle
};


struct by_angle {
    bool operator()(plane_struct const &a, plane_struct const &b) {
        return a.angle < b.angle;
    }
};

struct by_angle_line {
    bool operator()(line_struct const &a, line_struct const &b) {
        return a.angle < b.angle;
    }
};

struct by_distance {
    bool operator()(line_struct const &a, line_struct const &b) {
        return a.distance < b.distance;
    }
};


static bool abs_compare(int a, int b)
{
    return (std::abs(a) < std::abs(b));
}


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


  void draw_line(double* p0,double* p1);
  double interpolate ( double input , double input_start, double input_end, double output_start, double output_end);
  void draw_plane(double c0,double c1, double c2, double c3, double r, double g, double b);
  void draw_sphere(double x, double y, double z);
  double dot (row_type vec1, row_type vec2);
  row_type cross (row_type N_1, row_type N_2);
  double norm (row_type vec);
  row_type normalize (row_type);


public slots:

  virtual void slotExit();
  void on_loadFileButton_clicked();
  void on_clearButton_clicked();
  void on_unwrapButton_clicked();
  void on_colorComboBox_currentIndexChanged(const QString & text);
  void on_perspectiveCheckBox_clicked();
  void on_selectButton_clicked();
  void on_gridButton_clicked();

  void on_updateViewButton_clicked();


private:

  // Designer form
  Ui_Visualizer *ui;

  void clearAll();
  void updateView(int reset_camera=1);


};

#endif
