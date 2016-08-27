#ifndef MODEL_H
#define MODEL_H

#include <cmath>

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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/convolution.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>


#include <Eigen/Dense>

#include <QObject>


// #include <boost/ptr_container/ptr_vector.hpp>


// bool by_distance_center(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
//
//     if   (pcl::geometry::distance(p1, pcl::PointXYZ (0.0, 0.0, 0.0))  >  	pcl::geometry::distance(p2, pcl::PointXYZ (0.0, 0.0, 0.0)) ){
//         return true;
//     }else{
//         return false;
//     }
//         // if (p1.x < p2.x) {
//         //         return true;
//         // }
//         // else {
//         //         return false;
//         // }
// }



//Calculate the distance of a point int he cloud to the center (disregarding Z azis)
struct by_distance_center {
    bool operator()(pcl::PointXYZ const &p1, pcl::PointXYZ const &p2) {
        if   (pcl::geometry::distance(p1, pcl::PointXYZ (0.0, 0.0, p1.z))  >  	pcl::geometry::distance(p2, pcl::PointXYZ (0.0, 0.0, p2.z)) ){
            return true;
        }else{
            return false;
        }
    }
};


struct plane_struct {
    pcl::ModelCoefficients coef;
    Eigen::Vector3f normal;
    double angle;
    int index_cluster; //index to indicate which cluster is the plane resposible for
};


struct by_angle {
    bool operator()(plane_struct const &a, plane_struct const &b) {
        return a.angle < b.angle;
    }
};


struct column_comparer
{
    int column_num;
    column_comparer(int c) : column_num(c) {}

    bool operator()(const std::vector<double> & lhs, const std::vector<double> & rhs) const
    {
        return lhs[column_num] < rhs[column_num];
    }
};







class Model :  public QObject
{
     Q_OBJECT
public:
     Model();

     int m_num_walls;
     vtkSmartPointer<vtkPolyData> m_wall;


     //Wrapped
     matrix_type m_points_wrapped;
     matrix_type m_normals_wrapped;
     //  matrix_type m_tcoords_wrapped;
     vtkSmartPointer<vtkCellArray> m_cells_wrapped;
     pcl::PointCloud<pcl::PointXYZ>::Ptr m_points_wrapped_ds;

     //Unwrapped
     matrix_type m_points_unwrapped;
     matrix_type m_normals_unwrapped;
     vtkSmartPointer<vtkCellArray> m_cells_unwrapped;
     pcl::PointCloud<pcl::PointXYZ>::Ptr m_points_unwrapped_full_cloud;


     //Misc
     vtkSmartPointer<vtkTexture> m_full_texture;
     vtkSmartPointer<vtkUnsignedCharArray> m_colors_original; //original colors, rgb if present and all white if not
     vtkSmartPointer<vtkUnsignedCharArray> m_colors_active;
     row_type m_center;
     matrix_type m_streched_polys;
     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds;
     std::vector<plane_struct> planes;
     //  matrix_type m_walls_wrapped_bounds;
     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_original;
     matrix_type m_ransac_centers;


     //Unwrap results
      std::vector<pcl::PointIndices::Ptr> m_inliers_vec;



     //May need deleting
    //  vtkSmartPointer<vtkCellArray> m_cells;
     matrix_type m_normals;
     std::vector<double> m_angles;               //TODO: delete only if you are sure you're not going to use circular
     std::vector<double> m_distances_to_plane;
     std::vector<double> distances_to_radius;   //TODO: delete only if you are sure you're not going to use circular mesh
     std::vector<double> distances_to_center;   //TODO: delete only if you are sure you're not going to use circular mesh



    //  int m_num_points;        //num of points in the m_wall
    //  int m_point_components;  //components of each point, normally 3 (x,y,z)
     double m_radius;         //estimated radius of the cylinder
     double m_circumference;
     double* m_bounds;

     bool m_is_unwrapped;
     bool m_selecting_defects;
     bool m_selecting_grid;
     bool m_deleted_streached_trigs;
     bool m_draw_grid_active;
     bool m_draw_grid_inactive;

     //  std::vector<vtkSmartPointer<vtkPolyData>> grid_cells;
     matrix_type m_grid;
     std::vector<matrix_type> m_grid_wrapped;   //first index is the grid, second is the corner third is coordiante of corner
     row_type m_grid_cells_active;


     matrix_type m_plane_centers; //Used so that the view can get the centers so as to point the camera towards them


     void clear();
     void set_mesh(vtkSmartPointer<vtkPolyData>);
     void set_texture(vtkSmartPointer<vtkTexture>);
     void read_info();
     vtkSmartPointer<vtkUnsignedCharArray> get_colors();
     double estimate_radius (pcl::PointCloud<pcl::PointXYZ>::Ptr points );
     vtkSmartPointer<vtkPolyData> get_mesh();
     void write_points_to_mesh();
     void compute_unwrap();
     void compute_unwrap2();
     std::vector<double> compute_angles(matrix_type points);
     std::vector<double> compute_distances_to_radius(matrix_type points, double radius);
     void compute_plain_colors();
     void compute_rgb_colors();
     void compute_depth_colors();
     void create_grid();
     void scale_mesh();
     void center_mesh();
     void delete_streched_trigs();
     void add_streched_trigs();
     bool is_contained(pcl::PointXYZ , row_type);
     bool is_contained(row_type , row_type);
     pcl::PointCloud<pcl::PointXYZ>::Ptr compute_decimated(matrix_type);

     void blur_normals();

     double dist(row_type vec1, row_type vec2);

     void wrap_grid();
     void create_point_cloud();

public slots:
    void right_click_pressed_slot(row_type point);

signals:
    void grid_changed_signal();

};

#endif // MODEL_H
