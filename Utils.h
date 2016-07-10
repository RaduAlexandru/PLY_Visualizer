#ifndef UTILS_H
#define UTILS_H

#include <vtkSmartPointer.h>
#include <vtkPoints.h>

typedef std::vector<double> row_type;
typedef std::vector<row_type> matrix_type;


matrix_type vtk_to_vector(vtkSmartPointer<vtkPoints> points);
vtkSmartPointer<vtkPoints> vector_to_vtk(matrix_type points);

#endif
