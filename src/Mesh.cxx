#include "Mesh.h"
#include <iostream>

Mesh::Mesh():
  m_radius(0.0),
  m_circumference(0.0),
  m_num_walls_baked(0),
  m_num_walls(0),
  m_is_unwrapped(false),
  m_has_texture(false),
  m_has_ir(false),
  m_deleted_streached_trigs(false),
  m_has_tcoords(false),
  m_has_normals(false),
  m_is_obj(false),
  m_is_ply(false),
  m_selected_ir(false),
  m_is_cylindrical(false),
  m_wall(vtkSmartPointer<vtkPolyData>::New()),
  m_cells_wrapped(vtkSmartPointer<vtkCellArray>::New()),
  m_cells_unwrapped(vtkSmartPointer<vtkCellArray>::New()),
  m_tcoords_ir(vtkSmartPointer<vtkFloatArray>::New()),
  m_tcoords_rgb(vtkSmartPointer<vtkFloatArray>::New()),
  m_colors_original(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  m_colors_active(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  m_colors_bright(vtkSmartPointer<vtkUnsignedCharArray>::New()),

  m_texture_original(vtkSmartPointer<vtkTexture>::New()),
  m_texture_bright(vtkSmartPointer<vtkTexture>::New()),
  m_texture_ir(vtkSmartPointer<vtkTexture>::New())
{

}


void Mesh::set_mesh(vtkSmartPointer<vtkPolyData> polydata ){
  m_wall=auto_fix_pose(polydata);

  m_points_wrapped= vtk_to_vector(m_wall->GetPoints());
  m_points_wrapped_ds=compute_decimated(m_points_wrapped);
  m_cells_wrapped    =m_wall->GetPolys();
  m_bounds=m_wall->GetBounds();

  //get normals, the original, wrapped ones
  vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
  vtk_normals->SetNumberOfComponents(3);
  vtk_normals->SetName("Normals");
  vtk_normals = vtkFloatArray::SafeDownCast(m_wall->GetPointData()->GetNormals());
  if (vtk_normals){
    m_normals_wrapped=vtk_normal_tcoords_to_vector(vtk_normals);
  }else{
    std::cout << "Model::read_info: mesh does not have normals" << std::endl;
    //Estimate them
    vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
    normals_alg->SetInputConnection(m_wall->GetProducerPort());
    normals_alg->Update();
    vtkSmartPointer<vtkFloatArray> vtk_normals_estimated = vtkSmartPointer<vtkFloatArray>::New();
    vtk_normals_estimated->SetNumberOfComponents(3);
    vtk_normals_estimated->SetName("Normals");
    vtk_normals_estimated = vtkFloatArray::SafeDownCast(normals_alg->GetOutput()->GetPointData()->GetNormals());
    m_normals_wrapped=vtk_normal_tcoords_to_vector(vtk_normals_estimated);
    // m_wall->GetPointData()->SetNormals(vtk_normals_estimated);
  }

  //get radius and circumference
  m_radius           = estimate_radius(m_points_wrapped_ds);
  m_circumference    = estimate_circumference (m_radius, m_num_walls);

  //if it's ply get colors, both the original and the bright ones
  if (m_is_ply){
    m_colors_original  = get_colors();
    m_colors_active  = get_colors();

    m_colors_bright->SetNumberOfComponents(3);
    int num_colors=m_colors_original->GetNumberOfTuples();
    cv::Mat colors_bright(num_colors, 1,  CV_8UC3);

    //copy the pixels into a OpencvMat
    for (size_t i = 0; i < num_colors; i++) {
      cv::Vec3b &intensity = colors_bright.at<cv::Vec3b>(i, 0);
      double* color_pixel;
      color_pixel=m_colors_original->GetTuple(i);
      intensity.val[0] = color_pixel[0];
      intensity.val[1] = color_pixel[1];
      intensity.val[2] = color_pixel[2];
    }

    //brithen the OpencvMat
    cv::Mat hsv;
    cv::cvtColor(colors_bright,hsv,CV_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv,channels);
    cv::normalize(channels[1], channels[1], 0, 255, cv::NORM_MINMAX);
    cv::normalize(channels[2], channels[2], 0, 255, cv::NORM_MINMAX);
    cv::Mat result;
    cv::merge(channels,hsv);
    cv::cvtColor(hsv,result,CV_HSV2BGR);
    result.copyTo(colors_bright);

    //Copy colors bright back into a unsigned char arrays
    for (int i = 0; i < colors_bright.cols; i++) {
      for (int j = 0; j < colors_bright.rows; j++) {
          cv::Vec3b intensity = colors_bright.at<cv::Vec3b>(j, i);
          double color_pixel[3];
          color_pixel[0]= intensity.val[0];
          color_pixel[1]= intensity.val[1];
          color_pixel[2]= intensity.val[2];
          m_colors_bright->InsertNextTuple(color_pixel);
      }
    }
    // m_wall->GetPointData()->SetScalars(m_colors_bright);

  }

  if (m_is_obj){
    //read texture coordinates
    vtkSmartPointer<vtkFloatArray> vtk_tcoords = vtkSmartPointer<vtkFloatArray>::New();
    vtk_tcoords->SetNumberOfComponents(2);
    vtk_tcoords->SetName("Tcoords");
    vtk_tcoords = vtkFloatArray::SafeDownCast(m_wall->GetPointData()->GetTCoords());
    if(vtk_tcoords){
      std::cout << "Mesh::read_info: mesh has tcoords" << std::endl;
      m_has_tcoords=true;
      m_tcoords_rgb=vtk_tcoords;
    }else{
        std::cout << "Mesh::read_info: mesh does not have tcoords" << std::endl;
        m_has_tcoords=false;
    }

  }


  std::cout << "model::readinfo: num points= " <<m_points_wrapped.size() << std::endl;
  std::cout << "model::readinfo: nr cells: " <<  m_cells_wrapped->GetSize()  << std::endl;


}

double Mesh::estimate_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr points ){

  std::cout << "estimating radius" << std::endl;
  double radius=0.0;

  //Get the points that are further away
  std::sort(m_points_wrapped_ds->points.begin(), m_points_wrapped_ds->points.end(), by_distance_center());

  int further_num;
  further_num=50;
  //A proportion is more robust
  further_num=4*m_points_wrapped_ds->points.size()/100.0;
  std::vector< pcl::PointXYZ > further_points(m_points_wrapped_ds->points.begin(), m_points_wrapped_ds->points.begin() + further_num);


  //Get the median of those distance as the radius
  row_type distances(further_num);
  for (size_t i = 0; i < further_points.size(); i++) {
    float dist= utils::distance(further_points[i], pcl::PointXYZ (0.0, 0.0, further_points[i].z));
    distances[i]=dist;
  }

  radius= median(distances);

  std::cout << "radius is " << radius << std::endl;

  return radius;
}

double Mesh::estimate_circumference(double radius, double num_walls){
  double circumference;
  circumference    =2*M_PI*radius;   //In the case of a non cylindrical one, the circumference is the perimeter of all the walls
  double angle_wall=360.0/(double)num_walls;   // see law of cosines
  angle_wall= angle_wall *M_PI/180.0;
  circumference =num_walls * sqrt (radius*radius +radius*radius -2*radius*radius* cos (angle_wall));
  std::cout << "circumefrence is " << circumference << std::endl;
  return circumference;
}

vtkSmartPointer<vtkUnsignedCharArray> Mesh::get_colors(){

  vtkSmartPointer<vtkUnsignedCharArray> rgb= vtkSmartPointer<vtkUnsignedCharArray>::New();
  rgb->SetNumberOfComponents(3);
  rgb->SetName("original");

  int num       =m_wall->GetNumberOfPoints();
  auto scalars  =m_wall->GetPointData()->GetScalars();

  if (scalars==nullptr){
    std::cout << "no color" << std::endl;
    for (size_t i = 0; i < num; i++) {
      rgb->InsertNextTuple3(255.0, 255.0, 255.0);
    }
  }else{
    std::cout << "yes it has color " << std::endl;
    int num_colors=scalars->GetSize();

    //To avoid it inserting more colors than points
    int min= std::min (num_colors, num);
    for (vtkIdType i = 0; i < min; i++) {
      rgb->InsertNextTuple(scalars->GetTuple(i));
    }
  }

  // m_wall->GetPointData()->AddArray(rgb);

  return rgb;
}

vtkSmartPointer<vtkPolyData> Mesh::get_vtk_mesh(){
  if (m_points_wrapped.empty()){ //m_wall has not been set yet
    std::cout << "Mesh::get_vtk_mesh, there is no mesh set so returning an empty mesh" << '\n';
    return vtkSmartPointer<vtkPolyData>::New();
  }

  //Set points
  vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  if (this->m_is_unwrapped) points_active=vector_to_vtk(this->m_points_unwrapped);
  else                      points_active=vector_to_vtk(this->m_points_wrapped);


  std::cout << "Mesh::get_vtk_mesh colors active has size: "  << m_colors_active->GetNumberOfTuples() << '\n';

  m_wall->SetPoints(points_active);


  if (m_is_ply){
    m_wall->GetPointData()->SetScalars(m_colors_active);
  }
  std::cout << "Mesh get_vtk_mesh setting active colors with name" << m_colors_active->GetName()  << '\n';
  // m_wall->GetPointData()->SetActiveScalars("bright");


  if (this->m_is_unwrapped){
    delete_streched_trigs();
  }

  if (m_is_unwrapped){
    std::cout << "get_vtk_mesh, assigning  nr cells: " <<  m_cells_unwrapped->GetSize()  << std::endl;
    m_wall->SetPolys(m_cells_unwrapped);
  }else{
    std::cout << "get_vtk_mesh, assigning  nr cells: " <<  m_cells_wrapped->GetSize()  << std::endl;
    m_wall->SetPolys(m_cells_wrapped);
  }




  //Set the tcoords depending if ir is selected or not
  if(m_has_tcoords){
    if (m_selected_ir){
      std::cout << "writing ir tcoords" << std::endl;
      m_wall->GetPointData()->SetTCoords(m_tcoords_ir);
    }else{
      std::cout << "writing rgb tcoords" << std::endl;
      m_wall->GetPointData()->SetTCoords(m_tcoords_rgb);
    }
  }


  std::cout << "gettings bounds" << std::endl;
  this->m_bounds=m_wall->GetBounds();


  //TODO: we shoulnt set new normals but insted depending if its unwapped or not set the corresponding normals. compute_unwrap is the one that should calculate the new normals
  if (m_normals_wrapped.empty()) {
    std::cout << "get_mesh to mesh: NO NORMALS!!!!!" << std::endl;
    //Even if it doesnt have, estimate them
    vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
    normals_alg->SetInputConnection(m_wall->GetProducerPort());

    normals_alg->ComputePointNormalsOn();
    normals_alg->SplittingOff();
    normals_alg->ConsistencyOff();

    normals_alg->Update();

    vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
    vtk_normals->SetNumberOfComponents(3);
    vtk_normals->SetName("Normals");
    vtk_normals = vtkFloatArray::SafeDownCast(normals_alg->GetOutput()->GetPointData()->GetNormals());
    // vtk_normals=normals_alg->GetOutput()->GetPointData()->GetNormals();
    m_wall->GetPointData()->SetNormals(vtk_normals);


  }else{
    std::cout << "get mesh has normals" << std::endl;
    vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
    normals_alg->SetInputConnection(m_wall->GetProducerPort());

    normals_alg->ComputePointNormalsOn();
    normals_alg->SplittingOff();
    normals_alg->ConsistencyOff();

    std::cout << "updte" << std::endl;
    normals_alg->Update();

    std::cout << "11111" << std::endl;

    vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
    vtk_normals->SetNumberOfComponents(3);
    vtk_normals->SetName("Normals");
    vtk_normals = vtkFloatArray::SafeDownCast(normals_alg->GetOutput()->GetPointData()->GetNormals());
    // vtk_normals=normals_alg->GetOutput()->GetPointData()->GetNormals();

    std::cout << "setting new normals nr:" << vtk_normals->GetNumberOfTuples() << std::endl;
    m_wall->GetPointData()->SetNormals(vtk_normals);

  }


  std::cout << "finished wiriting points to mesh" << std::endl;
  return m_wall;
}

void Mesh::delete_streched_trigs(){
  //get all the cells and calculate the perimeter. If the perimeter excedes a certain value, delete the cell (poly)
  if (m_deleted_streached_trigs)
    return;
  m_deleted_streached_trigs=true;

  // std::cout << "delete streached trigs" << std::endl;
  m_wall->BuildLinks();
  vtkIdType n_pts=-1,*pts;
  m_wall->GetPolys()->InitTraversal();

  int counter=0;
  for (size_t i = 0; i < m_wall->GetPolys()->GetNumberOfCells(); i++) {
    m_wall->GetPolys()->GetNextCell(n_pts,pts);
    double p[3];
    matrix_type trig(3);
    for (size_t j = 0; j < 3; j++) {
      trig[j].resize(3);
    }
    for (size_t j = 0; j < 3; j++) {
      m_wall->GetPoints()->GetPoint(pts[j],trig[j].data());
      // std::cout << "Point " << i << " : (" << p[0] << " " << p[1] << " " << p[2] << ")" << std::endl;
    }

    //Now we have a triangle with 3 points
    double thresh=0.2;
    if (  utils::dist(trig[0], trig[1])  > thresh || utils::dist(trig[0], trig[2]) >thresh  || utils::dist(trig[2], trig[1]) >thresh ){
      // std::cout << "delete cell" << i << std::endl;
      m_wall->DeleteCell(i);
      counter++;
    }

  }

  m_wall->RemoveDeletedCells();
  m_cells_unwrapped=m_wall->GetPolys();
}

//TODO Implement it
vtkSmartPointer<vtkPolyData> Mesh::auto_fix_pose( vtkSmartPointer<vtkPolyData> polydata){

  vtkSmartPointer<vtkPolyData> wall_aligned;
  wall_aligned=polydata;

  return wall_aligned;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Mesh::compute_decimated(matrix_type points){

  std::cout << "started decimating nr: " << points.size()  << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width    = points.size();
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (points.size());

  for (size_t i = 0; i < points.size(); i++){
    cloud->points[i].x = points[i][0];
    cloud->points[i].y = points[i][1];
    cloud->points[i].z = points[i][2];
  }


  pcl::VoxelGrid<pcl::PointXYZ> ds;  //create downsampling filter
  ds.setInputCloud (cloud);
  ds.setLeafSize (0.07, 0.07, 0.07);
  ds.filter (*cloud);

  std::cout << "finished decimating with " << cloud->points.size() << std::endl;

    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
    // viewer.showCloud (cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

  return cloud;


}

void Mesh::compute_unwrap(){
  //need to get the circumference again because the number of walls may have changed and in the case of non conical, the circumference is the perimeter of the walls
  m_circumference    = estimate_circumference (m_radius, m_num_walls);

  //do some unwrap



  //calculate new normals for the unwrapped mesh
}



void Mesh::compute_plain_colors(){

  std::cout << "Mesh_compute plain colors--------------------------" << '\n';

  m_colors_active->Reset();
  m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active->SetNumberOfComponents(3);
  m_colors_active->SetName("plain");

  int num_points;
  if (m_is_unwrapped){
    num_points=m_points_unwrapped.size();
  }else{
    num_points=m_points_wrapped.size();
  }


  for (size_t i = 0; i < num_points; i++) {
    m_colors_active->InsertNextTuple3(255.0 ,255.0 ,255.0);
  }
  std::cout << "finishing creating colors" << std::endl;

  //m_wall->GetPointData()->SetScalars(colors_active);

}

void Mesh::compute_original_colors(){

  m_colors_active->Reset();
  m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active->SetNumberOfComponents(3);
  m_colors_active->SetName("original");


  int num_points;
  if (m_is_unwrapped){
    num_points=m_points_unwrapped.size();
  }else{
    num_points=m_points_wrapped.size();
  }

  int num_colors=m_colors_original->GetNumberOfTuples();
  int min = std::min (num_points, num_colors);
  for (vtkIdType i = 0; i < min; i++) {
    m_colors_active->InsertNextTuple(m_colors_original->GetTuple(i));
  }

  //m_wall->GetPointData()->SetScalars(colors_active);

}

void Mesh::compute_bright_colors(){

  std::cout << "Mesh::compute_bright_colors compute bright colors------------------" << '\n';

  m_colors_active->Reset();
  m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active->SetNumberOfComponents(3);
  m_colors_active->SetName("bright");


  int num_points;
  if (m_is_unwrapped){
    num_points=m_points_unwrapped.size();
  }else{
    num_points=m_points_wrapped.size();
  }

  int num_colors=m_colors_bright->GetNumberOfTuples();
  int min = std::min (num_points, num_colors);
  std::cout << "Mesh::compute_bright_colors num_colors is " << min << '\n';
  for (vtkIdType i = 0; i < min; i++) {
    m_colors_active->InsertNextTuple(m_colors_bright->GetTuple(i));
  }


}


void Mesh::compute_depth_colors(){



  // //FIRST WAY OF DOING IT- WORKED WITH THE CYLINDER
  // distances_to_radius = compute_distances_to_radius(m_points_wrapped, radius);
  //
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
  // colors_active->Reset();
  // colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  // colors_active->SetNumberOfComponents(3);
  // colors_active->SetName("depth");
  //
  //
  // std::cout << "starting to create colors" << std::endl;
  // for (size_t i = 0; i < num_points; i++) {
  //   //colors->InsertNextTuple3(angles[i]*255.0,0,0);
  //   colors_active->InsertNextTuple3(depth[i],depth[i],depth[i]);
  //   //m_points_unwrapped.InsertNextPoint( angles[i] *circumference,distances_from_radius[i],point[2])
  // }
  // std::cout << "finishing to create colors" << std::endl;
  // //FINISHED FIRST WAY---------------



  //Second way
  if (m_points_unwrapped.empty()){
    compute_unwrap(); //Compute unwrap to actually get unwrapped points
  }

  int num_points;
  if (m_is_unwrapped){
    num_points=m_points_unwrapped.size();
  }else{
    num_points=m_points_wrapped.size();
  }



  int column_num = 1;
  double max_dist = (*std::max_element(this->m_points_unwrapped.begin(), this->m_points_unwrapped.end(), column_comparer(column_num)))[column_num];
  double min_dist = (*std::min_element(this->m_points_unwrapped.begin(), this->m_points_unwrapped.end(), column_comparer(column_num)))[column_num];


  std::cout << "max, min dist is" << max_dist << " " << min_dist << std::endl;

  std::vector<double> depth(num_points);

  for (size_t i = 0; i < num_points; i++) {
    depth[i]=interpolate(this->m_points_unwrapped[i][1], min_dist, max_dist, 255.0, 0.0);
  }

  m_colors_active->Reset();
  m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active->SetNumberOfComponents(3);
  m_colors_active->SetName("depth");


  std::cout << "starting to create colors" << std::endl;
  for (size_t i = 0; i < num_points; i++) {
    //colors->InsertNextTuple3(angles[i]*255.0,0,0);
    m_colors_active->InsertNextTuple3(depth[i],depth[i],depth[i]);
    //m_points_unwrapped.InsertNextPoint( angles[i] *circumference,distances_from_radius[i],point[2])
  }
  std::cout << "finishing to create colors" << std::endl;


}



void Mesh::clear(){
  m_num_walls_baked = 0;
  m_num_walls = 0;
  m_is_unwrapped =false;
  m_has_texture = false;
  m_has_ir = false;
  m_deleted_streached_trigs = false;
  m_has_tcoords = false;
  m_has_normals = false;
  m_is_obj = false;
  m_is_ply= false;
  m_selected_ir = false;
  m_is_cylindrical = false;
  m_points_wrapped.clear();
}

void Mesh::set_state_ply(bool val){
  m_is_ply=val;
  m_is_obj=!val;
}
void Mesh::set_state_obj(bool val){
  m_is_ply=!val;
  m_is_obj=val;
}
void Mesh::set_state_selected_ir(bool val){
  m_selected_ir=val;
}
void Mesh::set_state_has_ir(bool val){
  m_has_ir=val;
}
void Mesh::set_state_unwrapped(bool val){
  m_is_unwrapped=val;
}
void Mesh::set_state_cylindrical(bool val){
  m_is_cylindrical=val;
}
void Mesh::set_nr_walls(int val){
  m_num_walls=val;
}
void Mesh::set_texture_bright(vtkSmartPointer<vtkTexture> texture){
  m_texture_bright=texture;
  m_has_texture=true;
}
void Mesh::set_texture_original(vtkSmartPointer<vtkTexture> texture){
  m_texture_original=texture;
  m_has_texture=true;
}
void Mesh::set_ir_texture(vtkSmartPointer<vtkTexture> texture){
  m_texture_ir=texture;
  m_has_texture=true;
}
bool Mesh::get_state_unwrapped(){
  return m_is_unwrapped;
}
int Mesh::get_nr_points_wrapped(){
  return m_points_wrapped.size();
}
bool Mesh::has_data(){
  return !m_points_wrapped.empty();
}
bool Mesh::has_texture(){
  return m_has_texture;
}
