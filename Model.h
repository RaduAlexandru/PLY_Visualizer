#ifndef MODEL_H
#define MODEL_H

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <math.h>
#include <iterator>
#include <algorithm>
#include <vtkClipPolyData.h>
#include <vtkBox.h>
#include <vtkPolyDataNormals.h>
#include <vtkImageData.h>
#include <vtkTexture.h>
#include "Utils.h"

typedef std::vector<double> row_type;
typedef std::vector<row_type> matrix_type;

class Model
{
public:
     Model();

     vtkSmartPointer<vtkPolyData> wall;
     vtkSmartPointer<vtkTexture> m_full_texture;
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
     double* bounds;

     bool is_unwrapped;
     bool selecting_defects;
     bool selecting_grid;

     std::vector<vtkSmartPointer<vtkPolyData>> grid_cells;

     void clear();
     void set_mesh(vtkPolyData*);
     void set_texture(vtkTexture*);
     void read_info();
     vtkSmartPointer<vtkUnsignedCharArray> get_colors();
     double estimate_radius (matrix_type points );
     vtkSmartPointer<vtkPolyData> get_mesh();
     void compute_unwrap();
     std::vector<double> compute_angles(matrix_type points);
     std::vector<double> compute_distances_to_radius(matrix_type points, double radius);
     double interpolate ( double input , double input_start, double input_end, double output_start, double output_end);
     void compute_plain_colors();
     void compute_rgb_colors();
     void compute_depth_colors();
     void create_grid();

};

#endif // MODEL_H
