#ifndef MODEL_H
#define MODEL_H

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

typedef std::vector<double> row_type;
typedef std::vector<row_type> matrix_type;

class Model
{
public:
     Model();

     vtkSmartPointer<vtkPolyData> wall;
     vtkSmartPointer<vtkCellArray> cells;
     vtkSmartPointer<vtkUnsignedCharArray> colors_original; //original colors, rgb if present and all white if not
     vtkSmartPointer<vtkUnsignedCharArray> colors_active;

     matrix_type points_wrapped;
     matrix_type points_unwrapped;

     std::vector<double> angles;
     std::vector<double> distances_to_radius;
     std::vector<double> distances_to_center;

     int num_points;        //num of points in the wall
     int point_components;  //components of each point, normally 3 (x,y,z)
     double radius;         //estimated radius of the cylinder
     double circumference;

     bool is_unwrapped;
     bool selecting_defects;
};

#endif // MODEL_H
