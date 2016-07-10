#include "Model.h"
#include <iostream>

Model::Model():
  num_points(0),
  point_components(0),
  radius(0.0),
  circumference(0.0),
  is_unwrapped(false),
  selecting_defects(false),
  selecting_grid(false),
  wall(vtkSmartPointer<vtkPolyData>::New()),
  cells(vtkSmartPointer<vtkCellArray>::New()),
  colors_original(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  colors_active(vtkSmartPointer<vtkUnsignedCharArray>::New())
{
  std::cout << "MODEL CONSTRUCT" << std::endl;

}


void Model::set_mesh(vtkPolyData* mesh){
  this->wall= mesh;
  clear();
  read_info();
}

void Model::clear(){
  this->is_unwrapped=false;
  this->selecting_defects=false;
  this->num_points=0;
  this->point_components=0;
  this->radius=0.0;
  this->circumference=0.0;
  this->points_wrapped.clear();
  this->points_unwrapped.clear();
  this->distances_to_radius.clear();
  this->distances_to_center.clear();
  this->angles.clear();
}


void Model::read_info(){
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points=wall->GetPoints();
  this->points_wrapped= vtk_to_vector(points);
  this->point_components = points->GetData()->GetNumberOfComponents();
  this->cells            =this->wall->GetPolys();
  this->colors_original  = get_colors();
  this->colors_active  = get_colors();
  this->num_points       =wall->GetNumberOfPoints();
  this->radius           = estimate_radius(points_wrapped);
  this->circumference    =2*M_PI*radius;

  this->bounds=wall->GetBounds();

  // for (size_t i = 0; i < 6; i++) {
  //   std::cout << "bounds: " << i << " is " << bounds[i] << std::endl;
  // }



}


vtkSmartPointer<vtkUnsignedCharArray> Model::get_colors(){

  vtkSmartPointer<vtkUnsignedCharArray> rgb= vtkSmartPointer<vtkUnsignedCharArray>::New();
  rgb->SetNumberOfComponents(3);

  int num       =wall->GetNumberOfPoints();
  auto scalars  =wall->GetPointData()->GetScalars();



  if (scalars==nullptr){
    std::cout << "no color" << std::endl;
    for (size_t i = 0; i < num; i++) {
      rgb->InsertNextTuple3(255.0, 255.0, 255.0);
    }
  }else{
    std::cout << "yes it has color " << std::endl;
    int num_colors=scalars->GetSize();
    std::cout << "number of values it has: " << num_colors << std::endl;

    //To avoid it inserting more colors than points
    int min= std::min (num_colors, num);

    for (vtkIdType i = 0; i < min; i++) {
      rgb->InsertNextTuple(scalars->GetTuple(i));

    }

  }

  rgb->SetName("rgb");

  wall->GetPointData()->AddArray(rgb);

  return rgb;

}

double Model::estimate_radius(matrix_type points ){
  std::cout << "esimating radius" << std::endl;

  int num_points=points.size();
  distances_to_center.resize(num_points);

  //no need to substract the center because we assume it's already centered
  //we do not use the z component since we suppose that the cylinder is oriented
  for (size_t i = 0; i < num_points; i++) {
    distances_to_center[i]= sqrt( points[i][0]*points[i][0]  + points[i][1]*points[i][1] );
  }


  //median of it will be the radius of the cylinder
  std::vector<double> calc(distances_to_center);
  size_t n = calc.size() / 2;
  std::nth_element(calc.begin(), calc.begin()+n, calc.end());
  double median= calc[n];
  std::cout << "radius estimated is:" << median << std::endl;
  return median;
}


vtkSmartPointer<vtkPolyData> Model::get_mesh(){
  std::cout << "model::updating mesh" << std::endl;

  if (num_points==0){ //Wall has not been set yet
    return wall;
  }

  //Set points
  vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  if (this->is_unwrapped){
    points_active=vector_to_vtk(this->points_unwrapped);
  }else{
    points_active=vector_to_vtk(this->points_wrapped);
  }

  wall->SetPoints(points_active);
  wall->SetPolys(this->cells);
  wall->GetPointData()->SetScalars(this->colors_active);

  this->bounds=wall->GetBounds();

  // for (size_t i = 0; i < 6; i++) {
  //   std::cout << "bounds: " << i << " is " << bounds[i] << std::endl;
  // }


  vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals_alg->SetInput(wall);
  normals_alg->Update();


  return normals_alg->GetOutput();
  //return wall;
}


void Model::compute_unwrap(){
  std::cout << "compute_unwrap" << std::endl;
  //Check if we have a wall
  //If we already have an unwrap for it, show it, otherwise, calculate it

  angles              =compute_angles(points_wrapped);
  distances_to_radius = compute_distances_to_radius(points_wrapped, radius);


  //assign the new points
  points_unwrapped.resize(num_points);
  for (size_t i = 0; i < num_points; i++) {
    points_unwrapped[i].resize(point_components);
    points_unwrapped[i][0]=angles[i] *circumference;
    points_unwrapped[i][1]=distances_to_radius[i];
    points_unwrapped[i][2]=points_wrapped[i][2];
  }
  std::cout << "finish asign points" << std::endl;

}


std::vector<double> Model::compute_angles(matrix_type points){


  //we again suppose that the center is already at 0,0,0
  int num_points=points.size();
  std::vector<double> angles;
  angles.resize(num_points);

  for (size_t i = 0; i < num_points; i++) {
    double ax=points[i][0];
    double ay=points[i][1];
    angles[i]=atan2(-ay,-ax);

    //interpolate it from -pi->pi to 0->1
    //angles[i] = 0.0 + ((1.0 - 0.0) / (3.1415 +3.1415)) * (angles[i] +3.1415);
    angles[i] = interpolate (  angles[i] , -M_PI, M_PI, 0.0, 1.0);
  }
  return angles;

}


std::vector<double> Model::compute_distances_to_radius(matrix_type points, double radius){
  int num_points=points.size();
  std::vector<double> dist;
  dist.resize(num_points);

  //Calculate first the distance to center.
  for (size_t i = 0; i < num_points; i++) {
    dist[i]= sqrt( points[i][0]*points[i][0]  + points[i][1]*points[i][1] );
    dist[i]= dist[i]-radius;
  }

  return dist;
}

double Model::interpolate ( double input , double input_start, double input_end, double output_start, double output_end){

  double output;
  output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);

  return output;

}




void Model::compute_plain_colors(){


  colors_active->Reset();
  colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors_active->SetNumberOfComponents(3);
  colors_active->SetName("plain");

  for (size_t i = 0; i < num_points; i++) {
    colors_active->InsertNextTuple3(255.0 ,255.0 ,255.0);
  }
  std::cout << "finishing to create colors" << std::endl;

  //wall->GetPointData()->SetScalars(colors_active);

}

void Model::compute_rgb_colors(){

  colors_active->Reset();
  colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors_active->SetNumberOfComponents(3);
  colors_active->SetName("rgb");

  for (vtkIdType i = 0; i < num_points; i++) {
    colors_active->InsertNextTuple(colors_original->GetTuple(i));
  }

  //wall->GetPointData()->SetScalars(colors_active);

}

void Model::compute_depth_colors(){

  distances_to_radius = compute_distances_to_radius(points_wrapped, radius);

  double max_dist=*(std::max_element(std::begin(distances_to_radius), std::end(distances_to_radius)));
  double min_dist=*(std::min_element(std::begin(distances_to_radius), std::end(distances_to_radius)));

  std::vector<double> depth;
  depth.resize(num_points);

  for (size_t i = 0; i < num_points; i++) {
    depth[i]=interpolate(distances_to_radius[i], min_dist, max_dist, 255.0, 0.0);
  }

  colors_active->Reset();
  colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors_active->SetNumberOfComponents(3);
  colors_active->SetName("depth");


  std::cout << "starting to create colors" << std::endl;
  for (size_t i = 0; i < num_points; i++) {
    //colors->InsertNextTuple3(angles[i]*255.0,0,0);
    colors_active->InsertNextTuple3(depth[i],depth[i],depth[i]);
    //points_unwrapped.InsertNextPoint( angles[i] *circumference,distances_from_radius[i],point[2])
  }
  std::cout << "finishing to create colors" << std::endl;

  //new_wall.GetPointData().SetScalars(new_colors)
  //wall->GetPointData()->SetScalars(colors_active);
}


void Model::create_grid(){

  std::cout << "model:creating grid" << std::endl;
  this->bounds=wall->GetBounds();

  //define the bounds of the box that will move around and cut. Size is 1x1m
  double box_size_x=40.0;
  double box_size_z=40.0;

  double box_bounds[6];  //xmin,xmax, ymin,ymax, zmin,zmax
  box_bounds[0]=0.0;
  box_bounds[1]=box_size_x;
  box_bounds[2]=-100000.0;  //the custom data I made is centered this way
  box_bounds[3]=+100000.0;
  box_bounds[4]=0.0;
  box_bounds[5]=box_size_z;


  int grid_size_x=ceil(bounds[1]/box_size_x);     //  xmax/box_size_x
  int grid_size_z=ceil(bounds[5]/box_size_z);     //  zmax/box_size_z

  std::cout << "The grid will have a total of " << grid_size_x*grid_size_z << " cells " << std::endl;

  for (size_t i = 0; i < grid_size_z; i++) {
    for (size_t j = 0; j < grid_size_x; j++) {
      //cut
      vtkSmartPointer<vtkBox> box_cut = vtkSmartPointer<vtkBox>::New();
      box_cut->SetBounds (box_bounds);
      vtkSmartPointer<vtkClipPolyData> clipper= vtkSmartPointer<vtkClipPolyData>::New();
      clipper->SetInputConnection(wall->GetProducerPort());
      clipper->SetClipFunction(box_cut);
      clipper->InsideOutOn();
      clipper->Update();
      //clipper->GenerateClippedOutputOn();

      std::cout << "cell has num of points" << clipper->GetOutput()->GetNumberOfPoints() << std::endl;


      //add polydata to the vector
      grid_cells.push_back(clipper->GetOutput());
      std::cout << "pushed a cell" << std::endl;

      //move box in x axis
      box_bounds[0]+=box_size_x;
      box_bounds[1]+=box_size_x;
    }
    //reset the box in the origin of x
    box_bounds[0]=0.0;
    box_bounds[1]=box_size_x;
    //move the box in the z axis
    box_bounds[4]+=box_size_z;
    box_bounds[5]+=box_size_z;
  }

  std::cout << "at the finale of crating grid, we have cells: " << grid_cells.size() << std::endl;

}
