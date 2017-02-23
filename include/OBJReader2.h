#ifndef OBJREADER2_H
#define OBJREADER2_H

#include <iterator>
#include <algorithm>
#include <sstream>
#include "Utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkQuadricDecimation.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>



typedef std::vector<double> row_type;
typedef std::vector<row_type> matrix_type;

typedef std::vector<int> row_type_i;
typedef std::vector<row_type_i> matrix_type_i;



class OBJReader2
{
public:
     OBJReader2();
     void SetFileName(std::string);
     void Update();
     std::string GetTexturePath();
     std::string GetTextureOriginalPath();
     vtkSmartPointer<vtkPolyData> GetOutput();

     bool experimental_loading;
     bool should_fix_orientation;

private:

     std::string m_full_texture_name;
     std::string m_full_texture_original_name;

     void read_mtl_file();
     void read_textures();
     void create_full_texture();
     void read_obj();
     void transform_tcoords();
     void write_to_poly();

     void fix_exposure();
     void fix_orientation();
     void auto_fix_orientation();
     void decimate_mesh();

     void for_each_pixel(cv::Mat &image, std::function<void(uchar * const pixel, int channels)> fn);


     std::string m_obj_file_name;
     std::string m_path;
     std::vector<std::string> m_texture_file_names;
     std::vector <cv::Mat> m_textures;
     cv::Mat m_full_texture;
     int m_multiplier;  //how big is the full texture with respect to the original ones.
     matrix_type m_tcoord_offsets;

     int m_indiv_texture_size;

     matrix_type m_points;
     matrix_type m_normals;
     matrix_type m_tcoords;
     std::vector <std::vector <matrix_type_i> > m_polys;   // vector of faces. First index is the material it belongs to. Second index is the actual face.  Each face is a matrix of 3x3 (3points each with 3 parameters: v,vt,vn)

     vtkSmartPointer<vtkPolyData> m_polyData;
};

#endif // OBJREADER2_H
