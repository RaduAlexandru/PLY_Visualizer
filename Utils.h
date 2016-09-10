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

#endif
