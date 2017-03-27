#ifndef UTILS_H
#define UTILS_H

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkDataArray.h>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>



typedef std::vector<double> row_type;
typedef std::vector<row_type> matrix_type;

typedef std::vector<int> row_type_i;
typedef std::vector<row_type_i> matrix_type_i;




  matrix_type vtk_to_vector(vtkSmartPointer<vtkPoints> points);
  vtkSmartPointer<vtkPoints> vector_to_vtk(matrix_type points);

  matrix_type vtk_normal_tcoords_to_vector(vtkSmartPointer<vtkDataArray>  vtk_normals);
  vtkSmartPointer<vtkFloatArray> vector_to_vtk_normal_tcoords( matrix_type normals);

  double interpolate ( double input , double input_start, double input_end, double output_start, double output_end);

  double median(row_type vec);

  std::string type2str(int type);


namespace utils {
  template <typename PointT> inline float distance (const PointT& p1, const PointT& p2) {
    Eigen::Vector3f diff;
    diff[0]=p1.x-p2.x;
    diff[1]=p1.y-p2.y;
    diff[2]=p1.z-p2.z;
    return (diff.norm ());
  }

  double dist(row_type vec1, row_type vec2);
  double dist_no_z(row_type vec1, row_type vec2);
}


#endif