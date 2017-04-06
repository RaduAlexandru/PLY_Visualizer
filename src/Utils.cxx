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



matrix_type vtk_normal_tcoords_to_vector(vtkSmartPointer<vtkDataArray>  vtk_normals){
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



vtkSmartPointer<vtkFloatArray> vector_to_vtk_normal_tcoords( matrix_type normals){


  vtkSmartPointer<vtkFloatArray> vtk_array = vtkSmartPointer<vtkFloatArray>::New();
  vtk_array->SetNumberOfComponents(3);
  // vtk_array->SetName("Normals");

  for (size_t i = 0; i < normals.size(); i++) {
    vtk_array->InsertNextTuple( normals[i].data());
  }

  return vtk_array;

}





double interpolate ( double input , double input_start, double input_end, double output_start, double output_end){

  double output;
  output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);

  return output;

}


double median(row_type vec) {
    if(vec.empty()) return 0;
    else {
        std::sort(vec.begin(), vec.end());
        if(vec.size() % 2 == 0)
                return (vec[vec.size()/2 - 1] + vec[vec.size()/2]) / 2;
        else
                return vec[vec.size()/2];
    }
}



std::string type2str(int type) {
  std::string r;

  unsigned char depth = type & CV_MAT_DEPTH_MASK;
  unsigned char chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


namespace utils{

  std::chrono::steady_clock::time_point begin;
  void tick(  ){
    begin = std::chrono::steady_clock::now();
  };

  // template<typename TimeT >
  // void tock(std::string name){
  //   std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
  //   std::cout << "TIME: " << name << " = " << std::chrono::duration_cast<TimeT>(end - begin).count() <<std::endl;
  // };



  double dist(row_type vec1, row_type vec2){
    double dist=0.0;
    for (size_t i = 0; i < vec1.size(); i++) {
      dist+= std::pow (vec1[i] - vec2[i],2);
    }
    dist=sqrt(dist);
    return dist;
  }

  double dist_squared(row_type vec1, row_type vec2){
    double dist=0.0;
    for (size_t i = 0; i < vec1.size(); i++) {
      dist+= std::pow (vec1[i] - vec2[i],2);
    }
    // dist=sqrt(dist);
    return dist;
  }


  double dist_no_z(row_type vec1, row_type vec2){
    double dist=0.0;
    for (size_t i = 0; i < vec1.size()-1; i++) {
      dist+= std::pow (vec1[i] - vec2[i],2);
    }
    dist=sqrt(dist);
    return dist;
  }

  double dist_no_y(row_type vec1, row_type vec2){
    double dist=0.0;

    dist+= std::pow (vec1[0] - vec2[0],2);
    dist+= std::pow (vec1[2] - vec2[2],2);
    
    dist=sqrt(dist);
    return dist;
  }

}
