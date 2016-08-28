#ifndef Visualizer_H
#define Visualizer_H

#include <vtkVersion.h>
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
// #include <vtkOBJImporter.h>
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
#include <vtkOutlineSource.h>
#include <vtkTextActor.h>
#include <vtkRenderLargeImage.h>
#include <vtkPNGWriter.h>
#include <vtkWorldPointPicker.h>
#include <vtkInteractorObserver.h>



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



#include <math.h>
#include <iterator>
#include <limits>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>


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
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>



#include <QFileDialog>
#include <QLabel>
#include <QMovie>
#include <QPushButton>
#include "Model.h"
#include "Utils.h"
#include "OBJReader2.h"
#include "InteractorPointPicker.h"
#include "ConfigDialog.h"

#include <QMainWindow>


// struct line_struct {
//     row_type direction;
//     row_type point;
//     double angle;   //angle of the line with respect to the x axis
//     double distance; //distance from a point to this line. Will be changed in each iteration when a point is checked
//     int index; //index assigned to indicat the ordering with respect to the angle
// };



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
  std::shared_ptr<ConfigDialog> m_config;

  void select_ir_mesh();

  void draw_grid();
  void draw_cell(row_type bounds, double r, double g, double b);
  void render_to_file(std::string path, int magnification=15);
  void SetCameraPositionOrientation( vtkCamera* cam, double position[3], double orientation[3] );
  void calculate_bounds( matrix_type corners, double* bounds);
  void world_to_display(double* point_world, double* point_display);


  //TODO: Remove it, just needed for testing the distance in knn
  void draw_sphere(double x, double y, double z,  double r=1.0, double g=1.0, double b=1.0);
  void draw_line (double* pos1, double* pos2 );

  void draw_text_grid();

  //Rendering to file
  void render_full_img();
  void render_grid_unwrapped();
  void render_grid_wrapped();
  void render_walls();



public slots:

  virtual void slotExit();
  void on_loadFileButton_clicked();
  void on_clearButton_clicked();
  void on_unwrapButton_clicked();
  void on_colorComboBox_currentIndexChanged(const QString & text);
  void on_perspectiveCheckBox_clicked();
  void on_selectButton_clicked();
  void on_showGridInactiveCheckBox_clicked();
  void on_showGridActiveCheckBox_clicked();
  void on_renderToImgButton_clicked();
  void on_renderGridCellButton_clicked();
  void on_renderWallsButton_clicked();
  void on_renderGridWrappedButton_clicked();
  void on_renderToFileButton_clicked();


  void on_experimentalLoadingcheckBox_clicked();
  void on_numWallsText_textChanged(const QString & text);
  void on_deformWallscheckBox_clicked();
  void on_clearUnwrapButton_clicked();
  void on_pathText_textChanged(const QString & text);
  void on_renderFullImgcheckBox_clicked();
  void on_renderGridUnwrappedcheckBox_clicked();
  void on_renderGridWrappedcheckBox_clicked();
  void on_renderWallscheckBox_clicked();
  void on_fullImgMagText_textChanged(const QString & text);
  void on_gridUnwrappedMagText_textChanged(const QString & text);
  void on_gridWrappedMagText_textChanged(const QString & text);
  void on_wallsMagText_textChanged(const QString & text);


  void on_loadFileConfigButton_clicked();

  void grid_changed_slot();



private:

  // Designer form
  Ui_Visualizer *ui;

  void clearAll();
  void updateView(int reset_camera=1);
  void update_grid_view();

  std::vector< vtkSmartPointer<vtkActor> > m_grid_actors;
  vtkSmartPointer<vtkTextActor> m_grid_metric_actor;


};

#endif
