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

#include "vtkTexturingHelper.h"



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


enum UpdateType { FULL_WALL, ONLY_COLOR, ONLY_CAMERA, ONLY_GRID };



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
  vtkSmartPointer<vtkPolyDataMapper> m_mapper_wall;
  vtkSmartPointer<vtkActor> m_actor_wall;
  vtkSmartPointer<InteractorPointPicker> interactor;
  std::shared_ptr<ConfigDialog> m_config;




  //reads a mesh that has IR texture. Sets that texture image and the IR texture coordinates into the interal mesh.
  void select_ir_mesh();
  //draws a grid of 1x1m^2 cells. It will draw the ones that are inactive in gray and the active ones (the ones selected) in red
  void draw_grid();
  //draw each individual cell of the grid.
  void draw_cell(row_type bounds, double r, double g, double b);
  //set the pose of the camera (TODO: check it again because it may not work as intendend since the orientation is indirectly set by modifying the focus)
  void set_cam_pose( vtkCamera* cam, double position[3], double orientation[3] );
  //Calculate the bounding box of a vector of points. It calculates the min and max in each dimension.
  void calculate_bounds( matrix_type corners, double* bounds);
  //projects a points in 3D world into the 2D image. The coordinate in 2D is relative to the upper left corner (the OpenCV standard)
  void world_to_display(double* point_world, double* point_display);
  void set_camera_default_pos();

  //orientate the mesh so that the princial axis of the chimney (the one running throungh the hole) is pointing in the Z direction of the world.
  vtkSmartPointer<vtkPolyData> auto_fix_orientation( vtkSmartPointer<vtkPolyData>);
  //void auto_fix_pose( vtkSmartPointer<vtkPolyData>);



  //DRAWING----------------------------------------------------------------------------------
  //TODO: Remove it, just needed for testing the distance in knn
  //Draw a sphere at a certain position in the world and with a certai color
  void draw_sphere(double x, double y, double z,  double r=1.0, double g=1.0, double b=1.0);
  //draw line between two points in the world
  void draw_line (double* pos1, double* pos2 );
  //draw a text with shows how many of the grid cells are selected (eg: 5/30)
  void draw_text_grid();

  //RENDER TO FILE--------------------------------------------------------------------------
  //render the image viewed on the screen into a file. The image will can be of a higher resolution than the screen depending on the magnification factor (1=using the screen resolution)
  void render_to_file(std::string path, int magnification=15);
  //set the camera in the correct position to render the full unwrapped wall. And then call render_to_file
  void render_full_img();
  //Move the camera around in front of each cell of the grid (cells are unwrapped). It then calls render_to_file
  void render_grid_unwrapped();
  //Move the camera around in front of each cell of the grid (cells are wrapped). It then calls render_to_file
  void render_grid_wrapped();
  //Move the camera around in front of wall (wrapped view). It then calls render_to_file
  void render_walls();



public slots:

  virtual void slotExit();
  void on_loadFileButton_clicked();
  void on_clearButton_clicked();
  void on_unwrapButton_clicked();
  void on_colorComboBox_currentIndexChanged(const QString & text);
  void on_flatShadingCheckBox_clicked();
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
  void on_fixOrientationcheckBox_clicked();
  void on_numWallsText_textChanged(const QString & text);
  void on_deformWallscheckBox_clicked();
  void on_clearUnwrapButton_clicked();
  void on_recomputeColorsButton_clicked();
  void on_pathText_textChanged(const QString & text);
  void on_renderFullImgcheckBox_clicked();
  void on_renderGridUnwrappedcheckBox_clicked();
  void on_renderGridWrappedcheckBox_clicked();
  void on_renderWallscheckBox_clicked();
  void on_fullImgMagText_textChanged(const QString & text);
  void on_gridUnwrappedMagText_textChanged(const QString & text);
  void on_gridWrappedMagText_textChanged(const QString & text);
  void on_wallsMagText_textChanged(const QString & text);
  void on_aboveThreshText_textChanged(const QString & text);
  void on_belowThreshText_textChanged(const QString & text);
  void on_highCapDepthColorSlider_valueChanged();
  void on_lowCapDepthColorSlider_valueChanged();


  void on_loadFileConfigButton_clicked();

  void grid_changed_slot();



private:

  // Designer form
  Ui_Visualizer *ui;

  void clearAll();
  // void updateView(int reset_camera=1);
  void updateView(UpdateType update_type=FULL_WALL);
  void update_grid_view();

  std::vector< vtkSmartPointer<vtkActor> > m_grid_actors;
  vtkSmartPointer<vtkTextActor> m_grid_metric_actor;


};

#endif
