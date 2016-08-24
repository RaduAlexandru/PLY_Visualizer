#ifndef UTILS_H
#define UTILS_H

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>





typedef std::vector<double> row_type;
typedef std::vector<row_type> matrix_type;

typedef std::vector<int> row_type_i;
typedef std::vector<row_type_i> matrix_type_i;


matrix_type vtk_to_vector(vtkSmartPointer<vtkPoints> points);
vtkSmartPointer<vtkPoints> vector_to_vtk(matrix_type points);

matrix_type vtk_normals_to_vector(vtkSmartPointer<vtkDataArray>  vtk_normals);

double interpolate ( double input , double input_start, double input_end, double output_start, double output_end);

double median(row_type vec);

#endif
