#ifndef MESH_H
#define MESH_H


#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <math.h>
#include <iterator>
#include <algorithm>
#include <vtkClipPolyData.h>
#include <vtkBox.h>
#include <vtkPolyDataNormals.h>
#include <vtkImageData.h>
#include <vtkTexture.h>
#include <vtkCleanPolyData.h>
#include <vtkPlaneSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkInteractorObserver.h>
#include <vtkOutlineSource.h>
#include <vtkTransform.h>
#include "Utils.h"

class Mesh {
public:
     Mesh();

     //
    //  void set_mesh(vtkSmartPointer<vtkPolyData>);
    //  void set_texture_bright(vtkSmartPointer<vtkTexture>);
    //  void set_texture_original(vtkSmartPointer<vtkTexture> texture);
    //  void set_ir_texture(vtkSmartPointer<vtkTexture>);
    //  vtkSmartPointer<vtkPolyData> get_vtk_mesh(); // returns a vtk mesh unwrapped or not
     //
    //  void compute_unwrap();
    //  void compute_plain_colors();
    //  void compute_rgb_colors();
    //  void compute_depth_colors();
    //  void compute_original_colors();


private:
     vtkSmartPointer<vtkPolyData> m_wall;
    //  matrix_type m_points_wrapped;
    //  matrix_type m_points_unwrapped;
    //  matrix_type m_normals_wrapped;
    //  matrix_type m_normals_unwrapped;
    //  vtkSmartPointer<vtkCellArray> m_cells_wrapped;
    //  vtkSmartPointer<vtkCellArray> m_cells_unwrapped;
    //  vtkSmartPointer<vtkDataArray> tcoords_ir;
    //  vtkSmartPointer<vtkDataArray> tcoords_rgb;
    //
    //
    //
    //
    //
    //  //State----------------Indicated the state of the mesh that is being displayed
    //  int m_num_walls_baked;  //the number of walls with which the m_points_unwrapped were computed
    //  bool m_is_unwrapped;  //is it is unwrapped then the get_mesh method should write the unwrapped points and vice-versa
    //  bool m_has_texture;
    //  bool m_has_ir;
    //  bool m_deleted_streached_trigs;
    //  bool m_has_tcoords;
    //  bool m_has_normals;
    //  bool m_is_obj;
    //  bool m_is_ply;
    //
    //
    //
    //  //Textures and colors
    //  vtkSmartPointer<vtkTexture> m_texture_original;
    //  vtkSmartPointer<vtkTexture> m_texture_bright;
    //  vtkSmartPointer<vtkTexture> m_ir_texture;
    //  vtkSmartPointer<vtkTexture> m_texture_active;
    //  vtkSmartPointer<vtkUnsignedCharArray> m_colors_original; //original colors, rgb if present and all white if not
    //  vtkSmartPointer<vtkUnsignedCharArray> m_colors_bright; //colors brightened
    //  vtkSmartPointer<vtkUnsignedCharArray> m_colors_active;
    //
    //
    //  //stuff needed to unwrap
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr m_points_wrapped_ds;
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr m_points_unwrapped_full_cloud;
    //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds;
    //  std::vector<plane_struct> m_planes;
    //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_clustered_clouds_original;
    //  matrix_type m_ransac_centers;
    //  matrix_type m_plane_centers; //Used so that the view can get the centers so as to point the camera towards them
    //
    //
    //  //Unwrap results
    //  std::vector<pcl::PointIndices::Ptr> m_inliers_vec;
    //  matrix_type m_normals_blured;
    //
    //
    //  //Grid
    //  matrix_type m_grid;
    //  std::vector<matrix_type> m_grid_wrapped;   //first index is the grid, second is the corner third is coordiante of corner
    //  row_type m_grid_cells_active;
    //
    //
    //  //Other stuff
    //  row_type m_center;
    //  double* m_bounds;
    //
    //
    //
    //
    //  //May need deleting
    //  std::vector<double> m_angles;               //TODO: delete only if you are sure you're not going to use circular
    //  std::vector<double> m_distances_to_plane;
    //  std::vector<double> m_distances_to_radius;   //TODO: delete only if you are sure you're not going to use circular mesh
    //  std::vector<double> m_distances_to_center;   //TODO: delete only if you are sure you're not going to use circular mesh
    //
    //
    //
    //
    //  void auto_fix_pose( vtkSmartPointer<vtkPolyData>);  //Fix the pose of the chimney so that the principal axis is along the Z axis and also centerd
    //  void compute_normals (vtkSmartPointer<vtkPolyData>);
    //  //Returns a code indicating if that particular container in the mesh is updated with the correct values or not. Meaning that unwrapped points may be computed into m_points_unwrapped but those points may not be written into the m_wall polydata
    // //  bool check_points_validity();
    // //  bool check_normals_validity();
    // //  bool check_colors_validity();
    // //  validate(); //updated in the vtk mesh whatever needs to be validated (eg, it may update the colors but not the points)
    //
    //
    //
    //  void clear();
    //  void read_info(int read_color=1);
    //  vtkSmartPointer<vtkUnsignedCharArray> get_colors();
    //
    //  //void write_points_to_mesh();
    //  void create_grid();
    //  void scale_mesh();
    //  void center_mesh();
    //  void delete_streched_trigs();
    //  bool is_contained(pcl::PointXYZ , row_type);
    //  bool is_contained(row_type , row_type);
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr compute_decimated(matrix_type);
    //  double dist(row_type vec1, row_type vec2);
    //  double dist_no_z(row_type vec1, row_type vec2);
    //  double find_angle(row_type p0, row_type p1, row_type p2); //angle between 3 points where p1 is the center
    //
    //  void blur_normals();
    //
    //
    //  void wrap_grid();
    //  void create_point_cloud();



};

#endif // MESH_H
