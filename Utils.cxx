#include "Utils.h"


matrix_type vtk_to_vector(vtkSmartPointer<vtkPoints> points){
  // std::cout << "vtk to vector" << std::endl;
  int num_components = points->GetData()->GetNumberOfComponents();
  // std::cout << "got components" << std::endl;
  int num_rows = points->GetData()->GetNumberOfTuples();

  // std::cout << "n_components is: " << num_components << std::endl;
  // std::cout << "n_rows is: " << num_rows << std::endl;

  row_type curTuple(num_components);
  matrix_type cpp_matrix(num_rows, row_type(num_components));

  for (int i=0; i<num_rows; i++) {
    points->GetData()->GetTuple(i, curTuple.data());
    cpp_matrix[i] = curTuple;
  }

  return cpp_matrix;
}

vtkSmartPointer<vtkPoints> vector_to_vtk(matrix_type points){
  // std::cout << "vector_to_vtk" << std::endl;
  // std:: cout << "v_to_vtk: size of points is: " << points.size() << std::endl;
  vtkSmartPointer<vtkPoints> points_vtk=  vtkSmartPointer<vtkPoints>::New();

  for (size_t i = 0; i < points.size(); i++) {
    //std::cout << "v_to_vtk: iter numer: "<< i  << std::endl;
    points_vtk->InsertNextPoint( points[i][0],points[i][1],points[i][2]);
    //points_vtk->InsertNextPoint( 0.0, 0.0, 0.0);
  }

  // std::cout << "finish vector_to_vtk" << std::endl;
  return points_vtk;
}



matrix_type vtk_normals_to_vector(vtkSmartPointer<vtkDataArray>  vtk_normals){
  std::cout << "vtk_normals to vector" << std::endl;

  int num_rows = vtk_normals->GetNumberOfTuples();
  int num_components = vtk_normals->GetNumberOfComponents();

  std::cout << "vtk-normals: n_components is: " << num_components << std::endl;
  std::cout << "vtk_normals: n_rows is: " << num_rows << std::endl;

  row_type curTuple(num_components);
  matrix_type cpp_matrix(num_rows, row_type(num_components));

  for (int i=0; i<num_rows; i++) {
    vtk_normals->GetTuple(i, curTuple.data());
    cpp_matrix[i] = curTuple;
  }

  return cpp_matrix;


}
