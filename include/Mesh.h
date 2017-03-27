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
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>

#include "Utils.h"

struct plane_struct {
    pcl::ModelCoefficients coef;
    Eigen::Vector3f normal;
    double angle;
    int index_cluster; //index to indicate which cluster is the plane resposible for
};

//distance of a point to the origin. Ignoring the z axis. We only want the distance in x and y
struct by_distance_center {
    bool operator()(pcl::PointXYZ const &p1, pcl::PointXYZ const &p2) {
        // if   (pcl::geometry::distance(p1, pcl::PointXYZ (0.0, 0.0, p1.z))  >  	pcl::geometry::distance(p2, pcl::PointXYZ (0.0, 0.0, p2.z)) ){
        //     return true;
        // }else{
        //     return false;
        // }

        //For some reason the old versio of pcl and/or eigen doesn't work with pcl::geometry::distance so I implementd my own distance calculation
        if   (utils::distance(p1, pcl::PointXYZ (0.0, 0.0, p1.z))  >  	utils::distance(p2, pcl::PointXYZ (0.0, 0.0, p2.z)) ){
            return true;
        }else{
            return false;
        }

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


class Mesh {
public:
     Mesh();


     void set_mesh(vtkSmartPointer<vtkPolyData>);
     void set_texture_bright(vtkSmartPointer<vtkTexture>);
     void set_texture_original(vtkSmartPointer<vtkTexture> texture);
     void set_ir_texture(vtkSmartPointer<vtkTexture>);
     vtkSmartPointer<vtkPolyData> get_vtk_mesh(); // returns a vtk mesh unwrapped or not depending on the set state
    //  vtkSmartPointer<vtkPolyData> get_unwrapped_mesh(); //Convenience method TODO: Implement
    //  vtkSmartPointer<vtkPolyData> get_wrapped_mesh(); //Convenience method TODO: Implement

     void compute_unwrap();
     void compute_plain_colors();
     void compute_bright_colors();
     void compute_depth_colors();
     void compute_original_colors();

     void set_state_ply(bool);
     void set_state_obj(bool);
     void set_state_selected_ir(bool);
     void set_state_has_ir(bool);
     void set_state_unwrapped(bool);
     void set_state_cylindrical(bool);
     void set_nr_walls(int);


     bool get_state_unwrapped();
     int get_nr_points_wrapped();
     vtkSmartPointer<vtkTexture> get_texture_original(){return m_texture_original;};
     vtkSmartPointer<vtkTexture> get_texture_bright(){return m_texture_bright;};
     vtkSmartPointer<vtkTexture> get_texture_ir(){return m_texture_ir;};
     bool is_ply(){return m_is_ply;};
     bool has_ir(){return m_has_ir;};
     bool has_data();
     bool has_texture();

     void clear();

private:
     vtkSmartPointer<vtkPolyData> m_wall;
     matrix_type m_points_wrapped;
     matrix_type m_points_unwrapped;
     matrix_type m_normals_wrapped;
     matrix_type m_normals_unwrapped;
     vtkSmartPointer<vtkCellArray> m_cells_wrapped;
     vtkSmartPointer<vtkCellArray> m_cells_unwrapped;
     vtkSmartPointer<vtkDataArray> m_tcoords_ir;
     vtkSmartPointer<vtkDataArray> m_tcoords_rgb;
     double m_radius;
     double m_circumference;



     //State----------------Indicated the state of the mesh that is being displayed
     int m_num_walls_baked;  //the number of walls with which the m_points_unwrapped were computed
     int m_num_walls;   //the number of walls indicated by the user. May not be the same that is already computed with the m_num_walls_baked
     bool m_is_unwrapped;  //is it is unwrapped then the get_mesh method should write the unwrapped points and vice-versa
     bool m_has_texture;
     bool m_has_ir;
     bool m_deleted_streached_trigs;
     bool m_has_tcoords;
     bool m_has_normals;
     bool m_is_obj;
     bool m_is_ply;
     bool m_selected_ir;
     bool m_is_cylindrical;



     //Textures and colors
     vtkSmartPointer<vtkTexture> m_texture_original;
     vtkSmartPointer<vtkTexture> m_texture_bright;
     vtkSmartPointer<vtkTexture> m_texture_ir;
     vtkSmartPointer<vtkUnsignedCharArray> m_colors_original; //original colors, rgb if present and all white if not
     vtkSmartPointer<vtkUnsignedCharArray> m_colors_bright; //colors brightened
     vtkSmartPointer<vtkUnsignedCharArray> m_colors_active;


     //stuff needed to unwrap
     pcl::PointCloud<pcl::PointXYZ>::Ptr m_points_wrapped_ds;
     pcl::PointCloud<pcl::PointXYZ>::Ptr m_points_unwrapped_full_cloud;
     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds;
     std::vector<plane_struct> m_planes;
     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_clustered_clouds_original;
     matrix_type m_ransac_centers;
     matrix_type m_plane_centers; //Used so that the view can get the centers so as to point the camera towards them


     //Unwrap results
     std::vector<pcl::PointIndices::Ptr> m_inliers_vec;
     matrix_type m_normals_blured;


     //Grid
     matrix_type m_grid;
     std::vector<matrix_type> m_grid_wrapped;   //first index is the grid, second is the corner third is coordiante of corner
     row_type m_grid_cells_active;


     //Other stuff
     row_type m_center;
     double* m_bounds;




     //May need deleting
     std::vector<double> m_angles;               //TODO: delete only if you are sure you're not going to use circular
     std::vector<double> m_distances_to_plane;
     std::vector<double> m_distances_to_radius;   //TODO: delete only if you are sure you're not going to use circular mesh
     std::vector<double> m_distances_to_center;   //TODO: delete only if you are sure you're not going to use circular mesh




     vtkSmartPointer<vtkPolyData> auto_fix_pose( vtkSmartPointer<vtkPolyData>);  //Fix the pose of the chimney so that the principal axis is along the Z axis and also centerd
     void compute_normals (vtkSmartPointer<vtkPolyData>);
     //Returns a code indicating if that particular container in the mesh is updated with the correct values or not. Meaning that unwrapped points may be computed into m_points_unwrapped but those points may not be written into the m_wall polydata
    //  bool check_points_validity();
    //  bool check_normals_validity();
    //  bool check_colors_validity();
    //  validate(); //updated in the vtk mesh whatever needs to be validated (eg, it may update the colors but not the points)



     vtkSmartPointer<vtkUnsignedCharArray> get_colors();  //read the color scalars of a mesh. If it has none it return a plain color
     double estimate_radius (pcl::PointCloud<pcl::PointXYZ>::Ptr points );
     double estimate_circumference(double radius, double num_walls);
     void create_grid();
     void scale_mesh();
     void center_mesh();
     void delete_streched_trigs(); //delete streched trigs from the unwrapped mesh and stores the cells for later use
     bool is_contained(pcl::PointXYZ , row_type);
     bool is_contained(row_type , row_type);
     pcl::PointCloud<pcl::PointXYZ>::Ptr compute_decimated(matrix_type);
    //  double dist(row_type vec1, row_type vec2);
    //  double dist_no_z(row_type vec1, row_type vec2);
     double find_angle(row_type p0, row_type p1, row_type p2); //angle between 3 points where p1 is the center

     void blur_normals();


     void wrap_grid();
     void create_point_cloud();





};

#endif // MESH_H
