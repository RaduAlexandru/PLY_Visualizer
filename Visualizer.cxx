#include "Visualizer.h"

// This is included here because it is forward declared in
// Visualizer.h
#include "ui_Visualizer.h"

#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>


#define PI 3.1415




//
// // Define interaction style
// class MouseInteractorStylePP : public vtkInteractorStyleTrackballCamera
// {
//   public:
//     //vtkSmartPointer<vtkPolyData> wall;
//
//
//     static MouseInteractorStylePP* New();
//     vtkTypeMacro(MouseInteractorStylePP, vtkInteractorStyleTrackballCamera);
//
//     virtual void OnLeftButtonDown()
//     {
//       std::cout << "Picking pixel: " << this->Interactor->GetEventPosition()[0] << " " << this->Interactor->GetEventPosition()[1] << std::endl;
//       this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0],
//                          this->Interactor->GetEventPosition()[1],
//                          0,  // always zero.
//                          this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
//       double picked[3];
//       this->Interactor->GetPicker()->GetPickPosition(picked);
//       std::cout << "Picked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;
//
//
//
//       //Clip a sphere out of it
//       vtkSmartPointer<vtkSphere> sphere_cut = vtkSmartPointer<vtkSphere>::New();
//       // box_cut->SetBounds (0.0, 1000, -0.005, 0.005, -1000.0, 1000.0);
//       //
//       // vtkSmartPointer<vtkClipPolyData> clipper= vtkSmartPointer<vtkClipPolyData>::New();
//       // clipper->SetInputConnection(reader->GetOutputPort());
//       // clipper->SetClipFunction(box_cut);
//       // clipper->InsideOutOff();
//       // clipper->Update();
//       // clipper->GenerateClippedOutputOn();
//
//
//
//
//
//
//       // Forward events
//       vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
//     }
//
//     // void OnMouseMove(){
//     //   std::cout << "mouse moving" << std::endl;
//     //   OnLeftButtonDown();
//     // }
//
// };
//
// vtkStandardNewMacro(MouseInteractorStylePP);



//vtkStandardNewMacro(InteractorPointPicker);







// Constructor
Visualizer::Visualizer():
  renderer(vtkSmartPointer<vtkRenderer>::New()),
  wall(vtkSmartPointer<vtkPolyData>::New()),
  points(vtkSmartPointer<vtkPoints>::New()),
  cells(vtkSmartPointer<vtkCellArray>::New()),
  colors(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  colors_original(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  interactor(vtkSmartPointer<InteractorPointPicker>::New())
{
  this->ui = new Ui_Visualizer;
  this->ui->setupUi(this);

  std::cout << "Visualizer constructor" << std::endl;

  this->ui->colorComboBox->addItem("Plain");
  this->ui->colorComboBox->addItem("RGB");
  this->ui->colorComboBox->addItem("Depth");
  this->ui->colorComboBox->addItem("Curvature");



  Model* model= new Model();

  // VTK Renderer
  renderer->GradientBackgroundOn();
  renderer->SetBackground(1.0,1.0,1.0);
  renderer->SetBackground2(0.1,0.1,0.1);
  // renderer->AddActor(sphereActor);


  // Changing interactor to a custom one to handle different mouse events
  vtkSmartPointer<vtkPointPicker> pointPicker = vtkSmartPointer<vtkPointPicker>::New();
  //interactor = vtkSmartPointer<InteractorPointPicker>::New();
  this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle( interactor );
  this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(pointPicker);



  renderer->GetActiveCamera()->SetParallelProjection(0);

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

  // Set up action signals and slots
  connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

}


void Visualizer::on_loadFileButton_clicked(){
  std::cout << "loading file" << std::endl;
  //renderer->RemoveAllViewProps();

  QString fileName;
	fileName = QFileDialog::getOpenFileName(this,
		tr("Open OBJ File"), "./", tr("OBJ File (*.ply)"));

	if (fileName.isEmpty())
		return;

  std::cout << "filename: " << fileName.toStdString() << std::endl;

  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( fileName.toStdString().c_str() );
  reader->Update();

  //cut the cylinder
  vtkSmartPointer<vtkBox> box_cut = vtkSmartPointer<vtkBox>::New();
  box_cut->SetBounds (0.0, 1000, -0.005, 0.005, -1000.0, 1000.0);

  vtkSmartPointer<vtkClipPolyData> clipper= vtkSmartPointer<vtkClipPolyData>::New();
  clipper->SetInputConnection(reader->GetOutputPort());
  clipper->SetClipFunction(box_cut);
  clipper->InsideOutOff();
  clipper->Update();
  clipper->GenerateClippedOutputOn();


  wall  = clipper->GetOutput();

  clearAll();
  getInfo(wall);
  updateView();
}

void Visualizer::clearAll(){

  this->is_unwrapped=FALSE;
  this->points_wrapped.clear();
  this->points_unwrapped.clear();
  this->distances_to_radius.clear();
  this->distances_to_center.clear();
  renderer->RemoveAllViewProps();
}

void Visualizer::getInfo(vtkSmartPointer<vtkPolyData> wall){
  this->points=wall->GetPoints();
  this->points_wrapped= vtk_to_vector(this->points);
  this->point_components = this->points->GetData()->GetNumberOfComponents();
  this->cells            =wall->GetPolys();
  //this->colors           =wall->GetPointData()->GetScalars();
  this->colors_original  = get_colors(wall);
  this->colors  = get_colors(wall);

  // auto test  =wall->GetPointData()->GetScalars();

  this->num_points       =wall->GetNumberOfPoints();
  this->radius           = estimateRadius(points_wrapped);
  this->circumference    =2*PI*radius;

  // std::cout << "getting colors" << std::endl;
  // if (test==NULL){
  //   std::cout << "no color" << std::endl;
  // }else{
  //   std::cout << "yes it has color " << std::endl;
  // }
  // std::cout << "number of values it has: " << test->GetSize() << std::endl;
  // for (vtkIdType i = 0; i < num_points; i++) {
  //   //std::cout << "touple::" << test->GetTuple(i)[0] << " " << test->GetTuple(i)[1]<< " "<<  test->GetTuple(i)[2] << std::endl;
  // }
  //   std::cout << "finishedgetting colors" << std::endl;
}

vtkSmartPointer<vtkUnsignedCharArray> Visualizer::get_colors(vtkSmartPointer<vtkPolyData> wall){

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

    for (vtkIdType i = 0; i < num_points; i++) {
      rgb->InsertNextTuple(scalars->GetTuple(i));
      //std::cout << "touple::" << scalars->GetTuple(i)[0] << " " << scalars->GetTuple(i)[1]<< " "<<  scalars->GetTuple(i)[2] << std::endl;
    }

  }

  rgb->SetName("rgb");

  wall->GetPointData()->AddArray(rgb);

  return rgb;


}

void  Visualizer::updateView(int reset_camera){
  std::cout << "update view" << std::endl;
  renderer->RemoveAllViewProps();

  //Set points
  vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  if (is_unwrapped){
    std::cout << "will create an unwrapped wall" << std::endl;
    std::cout << "points_unwrapped is: "<<points_unwrapped.size() << std::endl;
    points_active=vector_to_vtk(points_unwrapped);
  }else{
    std::cout << "will create an wrapped wall" << std::endl;
    std::cout << "points_wrapped is: "<<points_wrapped.size() << std::endl;
    points_active=vector_to_vtk(points_wrapped);
  }
  wall->SetPoints(points_active);
  wall->SetPolys(this->cells);
  wall->GetPointData()->SetScalars(colors);


  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(wall->GetProducerPort());
  mapper->Update();

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);


  renderer->AddActor(actor);
  if (reset_camera==1){
    renderer->ResetCamera();
  }

  interactor->wall=wall;
  //interactor->test=this->num_points;
  this->ui->qvtkWidget->GetRenderWindow()->Render();

}

void Visualizer::compute_unwrap(){
  std::cout << "compute_unwrap" << std::endl;
  //Check if we have a wall
  //If we already have an unwrap for it, show it, otherwise, calculate it

  this->angles              =computeAngles(this->points_wrapped);
  this->distances_to_radius = computeDistancesToRadius(this->points_wrapped, radius);


  //assign the new points
  points_unwrapped.resize(num_points);
  for (size_t i = 0; i < num_points; i++) {
    points_unwrapped[i].resize(point_components);
    points_unwrapped[i][0]=angles[i] *circumference;
    points_unwrapped[i][1]=distances_to_radius[i];
    points_unwrapped[i][2]=points_wrapped[i][2];
  }
  std::cout << "finish asign points" << std::endl;



  // double max_dist=*(std::max_element(std::begin(distances_to_radius), std::end(distances_to_radius)));
  // double min_dist=*(std::min_element(std::begin(distances_to_radius), std::end(distances_to_radius)));
  //
  // std::vector<double> depth;
  // depth.resize(num_points);
  //
  // for (size_t i = 0; i < num_points; i++) {
  //   depth[i]=interpolate(distances_to_radius[i], min_dist, max_dist, 255.0, 0.0);
  // }
  //
  // vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  // colors->SetNumberOfComponents(3);
  // colors->SetName("Colors");
  //
  //
  //
  //
  // std::cout << "starting to create colors" << std::endl;
  // for (size_t i = 0; i < num_points; i++) {
  //   //colors->InsertNextTuple3(angles[i]*255.0,0,0);
  //   colors->InsertNextTuple3(depth[i],depth[i],depth[i]);
  //   //points_unwrapped.InsertNextPoint( angles[i] *circumference,distances_from_radius[i],point[2])
  // }
  // std::cout << "finishing to create colors" << std::endl;
  //
  // //new_wall.GetPointData().SetScalars(new_colors)
  // wall->GetPointData()->SetScalars(colors);
}

void Visualizer::on_clearButton_clicked(){
  std::cout << "clearing the view" << std::endl;
  renderer->RemoveAllViewProps();
  std::cout << "clear: wall is poiting to" << wall << std::endl;
  this->ui->qvtkWidget->GetRenderWindow()->Render();
}

void Visualizer::on_unwrapButton_clicked(){
  std::cout << "unwrapping" << std::endl;


  if (!points_wrapped.empty()){
    this->is_unwrapped=!this->is_unwrapped;
  }

  if (points_unwrapped.empty() && !points_wrapped.empty()){
    compute_unwrap();
  }

  updateView();
}

void Visualizer::on_colorComboBox_currentIndexChanged(const QString & text){
  std::cout << text.toStdString() << std::endl;

  if (text.toStdString() == "Plain"){
    std::cout << "calculating plain colors" << std::endl;
    compute_plain_colors();
  }
  if (text.toStdString() == "RGB"){
    std::cout << "calculating RGB colors" << std::endl;
    compute_rgb_colors();
  }
  if (text.toStdString() == "Depth"){
    std::cout << "calculating Depth colors" << std::endl;
    compute_depth_colors();
  }
  if (text.toStdString() == "Curvature"){
    std::cout << "calculating Curvature colors" << std::endl;
  }

  updateView(0);

}

void Visualizer::on_perspectiveCheckBox_clicked(){
  std::cout << "change perspective" << std::endl;

  int current_val=renderer->GetActiveCamera()->GetParallelProjection();
  if (current_val==0)
    renderer->GetActiveCamera()->SetParallelProjection(1);
  else
    renderer->GetActiveCamera()->SetParallelProjection(0);

  updateView(0);

}

void Visualizer::compute_plain_colors(){


  colors->Reset();
  colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("plain");

  for (size_t i = 0; i < num_points; i++) {
    colors->InsertNextTuple3(255.0 ,255.0 ,255.0);
  }
  std::cout << "finishing to create colors" << std::endl;

  //new_wall.GetPointData().SetScalars(new_colors)
  wall->GetPointData()->SetScalars(colors);

}

void Visualizer::compute_rgb_colors(){

  colors->Reset();
  colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("plain");

  for (vtkIdType i = 0; i < num_points; i++) {
    colors->InsertNextTuple(colors_original->GetTuple(i));
    //colors->InsertNextTuple3(0.0 ,0.0 ,0.0);
  }

}

void Visualizer::compute_depth_colors(){

  this->distances_to_radius = computeDistancesToRadius(this->points_wrapped, radius);

  double max_dist=*(std::max_element(std::begin(distances_to_radius), std::end(distances_to_radius)));
  double min_dist=*(std::min_element(std::begin(distances_to_radius), std::end(distances_to_radius)));

  std::vector<double> depth;
  depth.resize(num_points);

  for (size_t i = 0; i < num_points; i++) {
    depth[i]=interpolate(distances_to_radius[i], min_dist, max_dist, 255.0, 0.0);
  }

  colors->Reset();
  colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("depth");




  std::cout << "starting to create colors" << std::endl;
  for (size_t i = 0; i < num_points; i++) {
    //colors->InsertNextTuple3(angles[i]*255.0,0,0);
    colors->InsertNextTuple3(depth[i],depth[i],depth[i]);
    //points_unwrapped.InsertNextPoint( angles[i] *circumference,distances_from_radius[i],point[2])
  }
  std::cout << "finishing to create colors" << std::endl;

  //new_wall.GetPointData().SetScalars(new_colors)
  wall->GetPointData()->SetScalars(colors);


}


double Visualizer::estimateRadius(matrix_type points ){
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


std::vector<double> Visualizer::computeAngles(matrix_type points){


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
    angles[i] = interpolate (  angles[i] , -PI, PI, 0.0, 1.0);
  }
  return angles;

}

std::vector<double> Visualizer::computeDistancesToRadius(matrix_type points, double radius){
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


matrix_type Visualizer::vtk_to_vector(vtkSmartPointer<vtkPoints> points){
  int num_components = points->GetData()->GetNumberOfComponents();
  int num_rows = points->GetData()->GetNumberOfTuples();

  std::cout << "n_components is: " << num_components << std::endl;
  std::cout << "n_rows is: " << num_rows << std::endl;

  row_type curTuple(num_components);
  matrix_type cpp_matrix(num_rows, row_type(num_components));

  for (int i=0; i<num_rows; i++) {
    points->GetData()->GetTuple(i, curTuple.data());
    cpp_matrix[i] = curTuple;
  }

  return cpp_matrix;
}

vtkSmartPointer<vtkPoints> Visualizer::vector_to_vtk(matrix_type points){
  std::cout << "vector_to_vtk" << std::endl;
  std:: cout << "v_to_vtk: size of points is: " << points.size() << std::endl;
  vtkSmartPointer<vtkPoints> points_vtk=  vtkSmartPointer<vtkPoints>::New();

  for (size_t i = 0; i < points.size(); i++) {
    //std::cout << "v_to_vtk: iter numer: "<< i  << std::endl;
    points_vtk->InsertNextPoint( points[i][0],points[i][1],points[i][2]);
    //points_vtk->InsertNextPoint( 0.0, 0.0, 0.0);
  }

  std::cout << "finish vector_to_vtk" << std::endl;
  return points_vtk;
}

double Visualizer::interpolate ( double input , double input_start, double input_end, double output_start, double output_end){

  double output;
  output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);

  return output;

}


void Visualizer::slotExit()
{
  qApp->exit();
}
