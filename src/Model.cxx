#include "Model.h"
#include <iostream>

Model::Model():
  mesh(new Mesh()),
  m_radius(0.0),
  m_circumference(0.0),
  m_is_unwrapped(false),
  m_selecting_defects(false),
  m_selecting_grid(false),
  m_wall(vtkSmartPointer<vtkPolyData>::New()),
  // m_cells(vtkSmartPointer<vtkCellArray>::New()),
  m_colors_original(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  m_colors_active(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  m_colors_bright(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  m_deleted_streached_trigs(false),
  m_draw_grid_active(true),
  m_draw_grid_inactive(false),
  m_has_ir(false),
  m_selected_ir(false),
  m_points_unwrapped_full_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  m_points_wrapped_ds(new pcl::PointCloud<pcl::PointXYZ>),


  m_cells_wrapped(vtkSmartPointer<vtkCellArray>::New()),
  m_cells_unwrapped(vtkSmartPointer<vtkCellArray>::New()),
  tcoords_ir(vtkSmartPointer<vtkFloatArray>::New()),
  tcoords_rgb(vtkSmartPointer<vtkFloatArray>::New()),

  //Options
  m_experiemental_loading(true),
  m_fix_orientation(true),
  m_num_walls(8),
  m_deform_walls(false),
  m_path_global(""),  //Will be set afterwards
  m_render_full_img(true),
  m_render_grid_unwrapped(true),
  m_render_grid_wrapped(true),
  m_render_walls(true),
  m_magnification_full_img(8),
  m_magnification_grid_unwrapped(2),
  m_magnification_grid_wrapped(2),
  m_magnification_walls(3),

  m_has_tcoords(false),
  m_has_normals(false),
  m_is_obj(false),
  m_is_ply(false),
  m_is_cylindrical(true)

{

  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;
  m_path_global=homedir;
  m_path_global+="/Pictures/Renders2/";
  std::cout << "home dir is " << m_path_global << std::endl;



}


void Model::set_mesh(vtkSmartPointer<vtkPolyData> mesh){
  clear();
  m_wall=mesh;
  center_mesh();
  read_info();
  //scale_mesh();  //TODO: REMOVE this one because it was only used so as to better see the fitted planes
}

void Model::set_texture_bright(vtkSmartPointer<vtkTexture> texture){
  m_texture_bright=texture;
}

void Model::set_texture_original(vtkSmartPointer<vtkTexture> texture){
  m_texture_original=texture;
}

void Model::set_ir_texture(vtkSmartPointer<vtkTexture> texture){
  m_ir_texture=texture;
}

void Model::clear(){
  m_is_unwrapped=false;
  m_selecting_defects=false;
  m_selecting_grid=false;
  m_deleted_streached_trigs=false;
  m_radius=0.0;
  m_circumference=0.0;
  m_points_wrapped.clear();
  m_points_unwrapped.clear();
  distances_to_radius.clear();
  distances_to_center.clear();
  m_angles.clear();
  m_grid_cells_active.clear();

  //-Options
  //Leave them as they are

  m_cells_wrapped->Initialize	();
  m_points_wrapped_ds->clear();
  m_cells_unwrapped->Initialize();
  m_points_unwrapped_full_cloud->clear();
  m_colors_original->Initialize();
  m_colors_active->Initialize();
  m_colors_bright->Initialize();

  m_colors_original= vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active= vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_bright= vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_center.clear();
  clustered_clouds.clear();
  planes.clear();
  clustered_clouds_original.clear();
  m_ransac_centers.clear();
  m_inliers_vec.clear();
  tcoords_ir->Initialize();
  tcoords_rgb->Initialize();
  m_normals_wrapped.clear();
  m_normals_unwrapped.clear();

  m_angles.clear();
  m_distances_to_plane.clear();
  distances_to_radius.clear();
  distances_to_center.clear();
  m_has_ir=false;
  m_selected_ir=false;
  m_grid.clear();
  m_grid_wrapped.clear();
  m_grid_cells_active.clear();
  m_plane_centers.clear();

  m_has_tcoords=false;
  m_has_normals=false;
  m_is_obj=false;
  m_is_ply=false;

  std::cout << "finished clearing" << std::endl;

}


void Model::read_info(int read_color){
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points=m_wall->GetPoints();
  m_points_wrapped= vtk_to_vector(points);
  m_points_wrapped_ds=compute_decimated(m_points_wrapped);
  m_cells_wrapped    =this->m_wall->GetPolys();
  if (read_color){
    m_colors_original  = get_colors();
    m_colors_active  = get_colors();
  }

  m_radius           = estimate_radius(m_points_wrapped_ds);
  m_circumference    =2*M_PI*m_radius;   //In the case of a non cylindrical one, the circumference is the perimeter of all the walls

  // see law of cosines
  double angle_wall=360.0/(double)m_num_walls;
  angle_wall= angle_wall *M_PI/180.0;
  m_circumference =m_num_walls * sqrt (m_radius*m_radius +m_radius*m_radius -2*m_radius*m_radius* cos (angle_wall));

  std::cout << "circumefrence is " << m_circumference << std::endl;


  //normals, the original, wrapped ones
  vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
  vtk_normals->SetNumberOfComponents(3);
  vtk_normals->SetName("Normals");
  vtk_normals = vtkFloatArray::SafeDownCast(m_wall->GetPointData()->GetNormals());

  if (vtk_normals){
    m_has_normals=true;
    m_normals_wrapped=vtk_normal_tcoords_to_vector(vtk_normals);
  }else{
    std::cout << "Model::read_info: mesh does not have normals" << std::endl;
    m_has_normals=false;
    //Estimate them

    vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
    #if VTK_MAJOR_VERSION <= 5
      normals_alg->SetInputConnection(m_wall->GetProducerPort());
    #else
      normals_alg->SetInputData(m_wall);
    #endif
    normals_alg->Update();
    vtkSmartPointer<vtkFloatArray> vtk_normals_estimated = vtkSmartPointer<vtkFloatArray>::New();
    vtk_normals_estimated->SetNumberOfComponents(3);
    vtk_normals_estimated->SetName("Normals");
    vtk_normals_estimated = vtkFloatArray::SafeDownCast(normals_alg->GetOutput()->GetPointData()->GetNormals());
    // vtk_normals=normals_alg->GetOutput()->GetPointData()->GetNormals();
    m_normals_wrapped=vtk_normal_tcoords_to_vector(vtk_normals_estimated);
    m_wall->GetPointData()->SetNormals(vtk_normals_estimated);
  }


  //texture coordinates. Not need to do anything with them so don't store them.
  vtkSmartPointer<vtkFloatArray> vtk_tcoords = vtkSmartPointer<vtkFloatArray>::New();
  vtk_tcoords->SetNumberOfComponents(2);
  vtk_tcoords->SetName("Tcoords");
  vtk_tcoords = vtkFloatArray::SafeDownCast(m_wall->GetPointData()->GetTCoords());
  if(vtk_tcoords){
    std::cout << "Model::read_info: mesh has tcoords" << std::endl;
    m_has_tcoords=true;
  }else{
      std::cout << "Model::read_info: mesh does not have tcoords" << std::endl;
      m_has_tcoords=false;
  }


  m_bounds=m_wall->GetBounds();


  std::cout << "model::readinfo: num points= " <<m_points_wrapped.size() << std::endl;
  std::cout << "model::readinfo: nr cells: " <<  m_cells_wrapped->GetSize()  << std::endl;
}


void Model::scale_mesh(){
  double scale=0.3;
  for (size_t i = 0; i < this->m_points_wrapped.size(); i++) {
    for (size_t j = 0; j < this->m_points_wrapped[0].size(); j++) {
      this->m_points_wrapped[i][j]= this->m_points_wrapped[i][j]*scale;
    }
  }
}


void Model::center_mesh(){

  // m_center.resize(m_point_components);
  //
  // m_center[0]=(m_bounds[1]+m_bounds[0])/2.0;
  // m_center[1]=(m_bounds[3]+m_bounds[2])/2.0;
  // m_center[2]=(m_bounds[5]+m_bounds[4])/2.0;
  //
  // //now we move the points to that center
  // for (size_t i = 0; i < this->m_points_wrapped.size(); i++) {
  //   for (size_t j = 0; j < this->m_points_wrapped[0].size(); j++) {
  //     this->m_points_wrapped[i][j]=this->m_points_wrapped[i][j]-m_center[j];
  //   }
  // }
  //
  // std::cout << "center is " <<   m_center[0] << " " << m_center[1] << " " << m_center [2] << std::endl;
  //
  // this->m_bounds=m_wall->GetBounds();
  std::cout << "centering mesh" << std::endl;

  m_bounds=m_wall->GetBounds();

  m_center.resize(3);
  m_center[0]=(m_bounds[1]+m_bounds[0])/2.0;
  m_center[1]=(m_bounds[3]+m_bounds[2])/2.0;
  m_center[2]=(m_bounds[5]+m_bounds[4])/2.0;


  vtkSmartPointer<vtkTransform> translation =
    vtkSmartPointer<vtkTransform>::New();
  translation->Translate(-m_center[0], -m_center[1], -m_center[2]);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

  #if VTK_MAJOR_VERSION <= 5
    transformFilter->SetInputConnection(m_wall->GetProducerPort());
  #else
    transformFilter->SetInputData(m_wall);
  #endif

  transformFilter->SetTransform(translation);
  transformFilter->Update();

  m_wall=transformFilter->GetOutput();

  // std::cout << "center is " <<   m_center[0] << " " << m_center[1] << " " << m_center [2] << std::endl;

}



vtkSmartPointer<vtkUnsignedCharArray> Model::get_colors(){

  vtkSmartPointer<vtkUnsignedCharArray> rgb= vtkSmartPointer<vtkUnsignedCharArray>::New();
  rgb->SetNumberOfComponents(3);

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

  rgb->SetName("rgb");

  m_wall->GetPointData()->AddArray(rgb);

  return rgb;
}

double Model::estimate_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr points ){

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


vtkSmartPointer<vtkPolyData> Model::get_mesh(){
  std::cout << "model::updating mesh" << std::endl;

  // write_points_to_mesh();
  return m_wall;

}

// void Model::write_points_to_mesh(){
//   if (m_points_wrapped.empty()){ //m_wall has not been set yet
//     return;
//   }
//
//   //Set points
//   vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
//   if (this->m_is_unwrapped){
//     points_active=vector_to_vtk(this->m_points_unwrapped);
//   }else{
//     points_active=vector_to_vtk(this->m_points_wrapped);
//   }
//   m_wall->SetPoints(points_active);
//
//   // if (m_colors_active->GetNumberOfTuples()>1){
//     m_wall->GetPointData()->SetScalars(this->m_colors_active);
//   // }
//
//   if (this->m_is_unwrapped){
//     delete_streched_trigs();
//   }
//
//   if (m_is_unwrapped){
//     m_wall->SetPolys(m_cells_unwrapped);
//   }else{
//     m_wall->SetPolys(m_cells_wrapped);
//   }
//
//   //Set the tcoords depending if ir is selected or not
//   if(m_has_tcoords){
//     if (m_selected_ir){
//       std::cout << "writing ir tcoords" << std::endl;
//       m_wall->GetPointData()->SetTCoords(tcoords_ir);
//     }else{
//       std::cout << "writing rgb tcoords" << std::endl;
//       m_wall->GetPointData()->SetTCoords(tcoords_rgb);
//     }
//   }
//
//
//   std::cout << "gettings bounds" << std::endl;
//   this->m_bounds=m_wall->GetBounds();
//
//
//   //TODO: we shoulnt set new normals but insted depending if its unwapped or not set the corresponding normals. compute_unwrap is the one that should calculate the new normals
//   if (m_normals_wrapped.empty()) {
//     std::cout << "write points to mesh: NO NORMALS!!!!!" << std::endl;
//     //Even if it doesnt have, estimate them
//     vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
//     #if VTK_MAJOR_VERSION <= 5
//       normals_alg->SetInputConnection(m_wall->GetProducerPort());
//     #else
//       normals_alg->SetInputData(m_wall);
//     #endif
//
//     normals_alg->ComputePointNormalsOn();
//     normals_alg->SplittingOff();
//     normals_alg->ConsistencyOff();
//
//     normals_alg->Update();
//
//     vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
//     vtk_normals->SetNumberOfComponents(3);
//     vtk_normals->SetName("Normals");
//     vtk_normals = vtkFloatArray::SafeDownCast(normals_alg->GetOutput()->GetPointData()->GetNormals());
//     // vtk_normals=normals_alg->GetOutput()->GetPointData()->GetNormals();
//     m_wall->GetPointData()->SetNormals(vtk_normals);
//
//
//   }else{
//     std::cout << "has normals" << std::endl;
//     vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
//     #if VTK_MAJOR_VERSION <= 5
//       normals_alg->SetInputConnection(m_wall->GetProducerPort());
//     #else
//       normals_alg->SetInputData(m_wall);
//     #endif
//
//     normals_alg->ComputePointNormalsOn();
//     normals_alg->SplittingOff();
//     normals_alg->ConsistencyOff();
//
//     std::cout << "updte" << std::endl;
//     normals_alg->Update();
//
//     std::cout << "11111" << std::endl;
//
//     vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
//     vtk_normals->SetNumberOfComponents(3);
//     vtk_normals->SetName("Normals");
//     vtk_normals = vtkFloatArray::SafeDownCast(normals_alg->GetOutput()->GetPointData()->GetNormals());
//     // vtk_normals=normals_alg->GetOutput()->GetPointData()->GetNormals();
//
//     std::cout << "setting new normals nr:" << vtk_normals->GetNumberOfTuples() << std::endl;
//     m_wall->GetPointData()->SetNormals(vtk_normals);
//
//   }
//
//
//   std::cout << "finished wiriting points to mesh" << std::endl;
//
// }
//
//



double Model::find_angle(row_type p0,row_type p1, row_type p2) {
  double a,b,c,res;

  a=std::pow(p1[0]- p0[0],2) + std::pow(p1[1] - p0[1],2);
  b=std::pow(p1[0]- p2[0],2) + std::pow(p1[1] - p2[1],2);
  c=std::pow(p2[0]- p0[0],2) + std::pow(p2[1] - p0[1],2);

  res=std::acos( (a+b-c) / std::sqrt(4*a*b) );

  return res;
}

void Model::blur_normals(){

  std::cout << "blurring the normals" << std::endl;
  m_normals_blured.resize(m_normals_wrapped.size());
  for (size_t i = 0; i < m_normals_blured.size(); i++) {
    m_normals_blured[i].resize(3);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZ>);

  cloud_normals->width    = m_normals_wrapped.size();
  cloud_normals->height   = 1;
  cloud_normals->is_dense = false;
  cloud_normals->points.resize (m_normals_wrapped.size());

  for (size_t i = 0; i < m_points_wrapped.size(); i++){
    cloud_normals->points[i].x = m_normals_wrapped[i][0];
    cloud_normals->points[i].y = m_normals_wrapped[i][1];
    cloud_normals->points[i].z = m_normals_wrapped[i][2];
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points(new pcl::PointCloud<pcl::PointXYZ>);

  cloud_points->width    = m_points_wrapped.size();
  cloud_points->height   = 1;
  cloud_points->is_dense = false;
  cloud_points->points.resize (m_points_wrapped.size());

  for (size_t i = 0; i < m_points_wrapped.size(); i++){
    cloud_points->points[i].x = m_points_wrapped[i][0];
    cloud_points->points[i].y = m_points_wrapped[i][1];
    cloud_points->points[i].z = m_points_wrapped[i][2];
  }


  // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  // viewer.showCloud (cloud_normals);
  // while (!viewer.wasStopped ())
  // {
  // }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud_points);

  double radius=0.05;
  // double radius=0.15;
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;


  row_type normal(3, 0.0);
  for (size_t i = 0; i < cloud_points->points.size(); i++) {
    if (  kdtree.radiusSearch (cloud_points->points[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
     normal[0]=cloud_normals->points[i].x;
     normal[1]=cloud_normals->points[i].y;
     normal[2]=cloud_normals->points[i].z;

     for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p){
        normal[0]+= cloud_normals->points[ pointIdxNKNSearch[p]].x;
        normal[1]+= cloud_normals->points[ pointIdxNKNSearch[p]].y;
        normal[2]+= cloud_normals->points[ pointIdxNKNSearch[p]].z;
      }

      normal[0]= normal[0]/pointIdxNKNSearch.size ();
      normal[1]= normal[1]/pointIdxNKNSearch.size ();
      normal[2]= normal[2]/pointIdxNKNSearch.size ();

      m_normals_blured[i][0] = normal[0];
      m_normals_blured[i][1] = normal[1];
      m_normals_blured[i][2] = normal[2];
    }
  }



  std::cout << "finished blurring the normals" << std::endl;

}


// void Model::compute_unwrap_cyl(){
//   //In this case the chimney may be cylindrical or even a cone
//   //calculate the distance of every point to the center. Plot the distance in Z and the radius of the points and you will see the line that corresponds with the radius
//
//   row_type center{0.0,0.0,0.0};
//   matrix_type data;  //data containing in the first coordinate the point and in the second coordinate the z and distance of the points
//   data.resize(m_points_wrapped.size());
//   for (size_t i = 0; i < m_points_wrapped.size(); i++) {
//     data[i].resize(2);
//   }
//
//   for (size_t i = 0; i < m_points_wrapped.size(); i++) {
//     double dist = dist_no_z(m_points_wrapped[i],center);
//     data[i][0]=m_points_wrapped[i][2]; //z coordinate of the point
//     data[i][1]=dist;
//     //std::cout << "dist is " << dist << '\n';
//   }
//
//   std::ofstream file("data.dat");
//   file << "#x y" << endl;
//   for(int i=0; i<data.size(); i++){
//       file << data[i][0] << ' ' << data[i][1] << endl;
//   }
//   file.close();
//
//   std::cout << "writing--------------------------------------------" << '\n';
//
//
// }
//
//
//





//TODO: When clearning up the code  I didn't even look at this so maybe you should sometime
//thi was of unwrapping it just runs runsac and then eliminates the inliers, then runs ransac again and again
void Model::compute_unwrap3(){

  //Show normals
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //
  //   cloud->width    = m_normals.size();
  //   cloud->height   = 1;
  //   cloud->is_dense = false;
  //   cloud->points.resize (m_normals.size());
  //
  //   for (size_t i = 0; i < cloud->points.size(); i++){
  //     cloud->points[i].x =  m_normals[i][0];
  //     cloud->points[i].y =  m_normals[i][1];
  //     cloud->points[i].z =  m_normals[i][2];
  //   }
  //
  //
  //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  //    viewer.showCloud (cloud);
  //    while (!viewer.wasStopped ())
  //    {
  //    }

  //Create cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width    = m_points_wrapped.size();
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (m_points_wrapped.size());

  for (size_t i = 0; i < cloud->points.size(); i++){
    cloud->points[i].x =  m_points_wrapped[i][0];
    cloud->points[i].y =  m_points_wrapped[i][1];
    cloud->points[i].z =  m_points_wrapped[i][2];
  }

  // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  // viewer.showCloud (cloud);
  // while (!viewer.wasStopped ())
  // {
  // }



  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.04);

  int i=0, nr_points = (int) cloud->points.size ();

  while (i<m_num_walls)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
    // viewer.showCloud (cloud_plane);
    // while (!viewer.wasStopped ())
    // {
    // }

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud = *cloud_f;

    i++;
  }



  std::cout << "remaining points " << cloud->points.size() << std::endl;

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }


}



void Model::compute_unwrap5(){
  //Run ransca to get intial planes
  //each point will get clustered into the plane that is closest to it.


  int cluster_count = m_num_walls;

  //vector of planes that will be ftted to the clouds
  planes.clear();
  planes.resize(m_num_walls);
  for (size_t i = 0; i < m_num_walls; i++) {
     planes[i].coef.values.resize(4);
     planes[i].index_cluster=i;
   }


  //Get the rough planes
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  seg.setOptimizeCoefficients (false);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (400);
  seg.setDistanceThreshold (0.03);

  int i=0, nr_points = (int) m_points_wrapped_ds->points.size ();

  while (i<m_num_walls)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (m_points_wrapped_ds);
    seg.segment (*inliers, (planes[i].coef));
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (m_points_wrapped_ds);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
    // viewer.showCloud (cloud_plane);
    // while (!viewer.wasStopped ())
    // {
    // }

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *m_points_wrapped_ds = *cloud_f;

    i++;
  }


  //Add the normal as an eigen vector for easier calculations
  for (size_t i = 0; i < planes.size(); i++) {
    planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
  }


  //Fix normals of planes so that they point towards the center
  //Dot product between the (point on a plane- center) and the normal (if it's negative then flip the normal)
  //WATCHOUT I SET Z and Y to 0 because if I set x and y Imight not get a solution since the plane might be paralel to Z axis.
  //Needs a more reobust way of finding points to account for parallel planes
  for (size_t i = 0; i < planes.size(); i++) {
    Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ) - m_center[0],
                        - m_center[1],
                        - m_center[2]);

    double dot= planes[i].normal.dot(dir);

    if (dot<0.0){
      std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
      planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
    }
  }



  //get the angles of the planes with respect to the x axis
  for (size_t i = 0; i < planes.size(); i++) {
      Eigen::Vector2f normal = Eigen::Vector2f::Map(planes[i].coef.values.data(), 2);
      Eigen::Vector2f x_axis (1.0, 0.0);


      float dot = normal.dot(x_axis);      //dot produt
      float cross = normal.x()*x_axis.y() - x_axis.x()*normal.y(); //cross, determinant
      double angle = atan2(cross, dot) ;  // atan2(y, x) or atan2(sin, cos)

      angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);

      planes[i].angle=angle;
  }
  std::sort(planes.begin(), planes.end(), by_angle());

  for (size_t i = 0; i < planes.size(); i++) {
    planes[i].index_cluster=i;
    planes[i].coef.values[3]=planes[i].coef.values[3]-0.1;  //HACK
  }


 //-------------------

  matrix_type_i clustered_points_indices(m_num_walls);


  //Build a vector of cloud points
  clustered_clouds.clear();
  for (size_t clust = 0; clust < m_num_walls; clust++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   clustered_clouds.push_back(cloud);
  }




  int clust_idx;
  //For each point in the mesh get the plane that is closest to it.
  for (size_t i = 0; i < m_points_wrapped.size(); i++) {

    int closest_plane_idx=-1;
    double distance=999999999;

    for (size_t plane_idx = 0; plane_idx < planes.size(); plane_idx++) {
      pcl::PointXYZ p;
      p.x=m_points_wrapped[i][0];
      p.y=m_points_wrapped[i][1];
      p.z=m_points_wrapped[i][2];
      double cur_dist=pcl::pointToPlaneDistanceSigned (p , planes[plane_idx].coef.values[0],
                                                           planes[plane_idx].coef.values[1],
                                                           planes[plane_idx].coef.values[2],
                                                           planes[plane_idx].coef.values[3]);

      cur_dist=std::fabs(cur_dist);
      if (cur_dist < distance){
        distance= cur_dist;
        closest_plane_idx=plane_idx;
      }
    }


    pcl::PointXYZ pt;
    pt.x=m_points_wrapped[i][0];
    pt.y=m_points_wrapped[i][1];
    pt.z=m_points_wrapped[i][2];


    clust_idx=planes[closest_plane_idx].index_cluster;
    clustered_clouds[clust_idx]->points.push_back(pt);
    clustered_points_indices[clust_idx].push_back(i);
  }

  std::cout << "finished clutering points" << std::endl;

  for (size_t i = 0; i < m_num_walls; i++) {
    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
    // viewer.showCloud (clustered_clouds[i]);
    // while (!viewer.wasStopped ())
    // {
    // }
  }


  //-----------------------




  //NOW we have the clutered clouds


  //Decimate the clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
  for (size_t clust = 0; clust < cluster_count; clust++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    clustered_clouds_ds.push_back(cloud);
  }
  for (size_t clust = 0; clust < cluster_count; clust++) {
    pcl::VoxelGrid<pcl::PointXYZ> ds;  //create downsampling filter
    ds.setInputCloud (clustered_clouds[clust]);
    ds.setLeafSize (0.1, 0.1, 0.1);
    ds.filter (*clustered_clouds_ds[clust]);
  }




  //vector of planes that will be ftted to the clouds
  planes.clear();
  planes.resize(cluster_count);
  for (size_t i = 0; i < cluster_count; i++) {
     planes[i].coef.values.resize(4);
     planes[i].index_cluster=i;
   }


  std::cout << "segmenting" << std::endl;

  m_inliers_vec.clear();

  #pragma omp parallel for
  for (size_t clust = 0; clust < cluster_count; clust++) {
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg2;

    //  seg.setOptimizeCoefficients (true);

    seg2.setModelType (pcl::SACMODEL_PLANE);
    seg2.setMethodType (pcl::SAC_RANSAC);
    //  seg.setDistanceThreshold (0.02);
    seg2.setDistanceThreshold (0.05);
   // seg.setDistanceThreshold (0.007);

    seg2.setInputCloud (clustered_clouds[clust]);
    seg2.segment (*inliers2, (planes[clust].coef));

    if (inliers2->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    m_inliers_vec.push_back(inliers2);


    std::cerr << "Model coefficients: " << planes[clust].coef.values[0] << " "
                                         << planes[clust].coef.values[1] << " "
                                         << planes[clust].coef.values[2] << " "
                                         << planes[clust].coef.values[3] << std::endl;
 }


 //Add the normal as an eigen vector for easier calculations
 for (size_t i = 0; i < planes.size(); i++) {
   planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
 }


 //Fix normals of planes so that they point towards the center
 //Dot product between the (point on a plane- center) and the normal (if it's negative then flip the normal)
 //WATCHOUT I SET Z and Y to 0 because if I set x and y Imight not get a solution since the plane might be paralel to Z axis.
 //Needs a more reobust way of finding points to account for parallel planes
 for (size_t i = 0; i < planes.size(); i++) {
   Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ) - m_center[0],
                       - m_center[1],
                       - m_center[2]);

   double dot= planes[i].normal.dot(dir);

   if (dot<0.0){
     std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
     planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
   }
 }



 //get the angles of the planes with respect to the x axis
 for (size_t i = 0; i < planes.size(); i++) {
     Eigen::Vector2f normal = Eigen::Vector2f::Map(planes[i].coef.values.data(), 2);
     Eigen::Vector2f x_axis (1.0, 0.0);


     float dot = normal.dot(x_axis);      //dot produt
     float cross = normal.x()*x_axis.y() - x_axis.x()*normal.y(); //cross, determinant
     double angle = atan2(cross, dot) ;  // atan2(y, x) or atan2(sin, cos)

     angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);

     planes[i].angle=angle;
 }
 std::sort(planes.begin(), planes.end(), by_angle());




 //Writing the original walls in the model
 // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
 clustered_clouds_original.clear();
 for (size_t clust = 0; clust < cluster_count; clust++) {
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   clustered_clouds_original.push_back(cloud);
 }
 for (size_t clust = 0; clust < cluster_count; clust++) {
   clustered_clouds_original[clust]->width    = clustered_clouds[clust]->points.size();
   clustered_clouds_original[clust]->height   = 1;
   clustered_clouds_original[clust]->is_dense = false;
   clustered_clouds_original[clust]->points.resize (clustered_clouds[clust]->points.size());

   for (size_t i = 0; i < clustered_clouds[clust]->points.size(); i++){
     clustered_clouds_original[clust]->points[i].x =  clustered_clouds[clust]->points[i].x;
     clustered_clouds_original[clust]->points[i].y =  clustered_clouds[clust]->points[i].y;
     clustered_clouds_original[clust]->points[i].z =  clustered_clouds[clust]->points[i].z;
   }

 }









 // go through the planes and calculate the rotation translation matrix to map them
 m_plane_centers.resize(planes.size());
 #pragma omp parallel for
 for (size_t i = 0; i < planes.size(); i++) {
   //TODO::MAKE ALL this code use to use eigen, clean it

   vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
   planeSource->SetCenter(0.0, 0.0, 0.0);
   planeSource->SetNormal( planes[i].coef.values[0],
                           planes[i].coef.values[1],
                           planes[i].coef.values[2]);
   planeSource->Push(-planes[i].coef.values[3]);
   planeSource->Update();

   double center_vtk[3];
   planeSource->GetCenter(center_vtk);
   m_plane_centers[i]=row_type {center_vtk[0],center_vtk[1],center_vtk[2]};
   // Eigen::Vector3f plane_center = Eigen::Vector3f::Map(center_vtk, 3);
   // plane_center[2]=0.0;

   Eigen::Vector3f plane_center;
   plane_center[0]=center_vtk[0];
   plane_center[1]=center_vtk[1];
   plane_center[2]=0.0;


   // double offset= 0.03*i;  //TODO REMOVE: Each plane gets closer and closer
   // double offset= 0.05*i;  //TODO REMOVE: Each plane gets closer and closer
   double offset= 0.00*i;  //TODO REMOVE: Each plane gets closer and closer


   row_type end_point(3);
   end_point[0]= planes[i].angle * m_circumference - offset; //I don't know why it needs to be summed and not -
   end_point[1]= 0.0;
   end_point[2]= 0.0;


   //TEST: Second way of calculating the end point
   // double angle_wall=360.0/(double)m_num_walls;
   // angle_wall= angle_wall *M_PI/180.0;
   // double side= sqrt (m_radius*m_radius +m_radius*m_radius -2*m_radius*m_radius* cos (angle_wall));
   // // pos=pos/2;
   // double pos= side*i;
   // pos=pos- (side/2);
   // end_point[0]=pos;
   //-----------------

   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
   Eigen::Affine3f transform_center = Eigen::Affine3f::Identity();

   // Define a translation of 2.5 meters on the x axis.
   transform_2.translation() << end_point[0], end_point[1], end_point[2];
   transform_center.translation() << -plane_center[0], -plane_center[1], -plane_center[2];


   double r = (90 * M_PI /180) + interpolate (planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
   transform_2.rotate (Eigen::AngleAxisf (r, Eigen::Vector3f::UnitZ()));


   std::cout << transform_2.matrix() << std::endl;
   int clust_idx= planes[i].index_cluster;

   pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform_center);
   pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform_2);

 }




 //Move the walls closer together
 matrix_type min_max(cluster_count);
 // for (size_t clust = 0; clust < cluster_count; clust++) {
 //   //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
 //   double min_x_cur=std::numeric_limits<double>::max();;
 //   double max_x_cur=std::numeric_limits<double>::min();;
 //   for (size_t i_idx = 0; i_idx < m_inliers_vec[clust]->indices.size (); i_idx++) {
 //     if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x > max_x_cur  ){
 //       max_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
 //     }
 //     if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x < min_x_cur  ){
 //       min_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
 //     }
 //
 //   }
 //   // std::cout << "cluster " << clust  << std::endl;
 //   // std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;
 //
 //   min_max[clust]=row_type {min_x_cur, max_x_cur};  //there are now unordered
 // }


 for (size_t clust = 0; clust < cluster_count; clust++) {
   //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
   double min_x_cur=std::numeric_limits<double>::max();;
   double max_x_cur=std::numeric_limits<double>::min();;
   for (size_t i_idx = 0; i_idx < clustered_clouds[clust]->points.size (); i_idx++) {
     if ( clustered_clouds[clust]->points[i_idx].x > max_x_cur  ){
       max_x_cur=clustered_clouds[clust]->points[i_idx].x;
     }
     if ( clustered_clouds[clust]->points[i_idx].x < min_x_cur  ){
       min_x_cur=clustered_clouds[clust]->points[i_idx].x;
     }

   }
   // std::cout << "cluster " << clust  << std::endl;
   // std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;

   min_max[clust]=row_type {min_x_cur, max_x_cur};  //there are now unordered
 }


 //Now that we have the min and max, move them
 std::cout << "movements" << std::endl;
 row_type movements(cluster_count, 0.0);
 for (size_t i = 0; i < planes.size(); i++) {
   int clust_idx= planes[i].index_cluster;
   if (i==0){
     movements[clust_idx]= 0.0 - min_max[clust_idx][0];
   }else{
     movements[clust_idx]=0.0;
     movements[clust_idx]+= movements[planes[i-1].index_cluster];
     movements[clust_idx]+= (min_max[planes[i-1].index_cluster][1] - min_max[clust_idx][0] );
   }
   std::cout << "clust index " << clust_idx << std::endl;
   std::cout << "movemnt: " << movements[clust_idx] << std::endl;
 }

 std::cout << "move them given the min and max" << std::endl;
 for (size_t i = 0; i < planes.size(); i++) {
   int clust_idx= planes[i].index_cluster;



   std::cout << "plane: " << i << std::endl;
   std::cout << "min max: " << min_max[clust_idx][0] << " " << min_max[clust_idx][1] << std::endl;



   //Move it
   // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
   // transform.translation() << min_x_cur - max_x_prev, 0.0, 0.0;
   // pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform);
   for (size_t p_idx = 0; p_idx < clustered_clouds[clust_idx]->points.size(); p_idx++) {
     clustered_clouds[clust_idx]->points[p_idx].x = clustered_clouds[clust_idx]->points[p_idx].x + movements[clust_idx];
   }

 }


 //showing stuff
 // for (size_t i = 0; i < planes.size(); i++) {
 //   int clust_idx= planes[i].index_cluster;
 //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
 //    viewer.showCloud (clustered_clouds[clust_idx]);
 //    while (!viewer.wasStopped ())
 //    {
 //    }
 // }




 std::cout << "wiritng to unwrapped points" << std::endl;

 //write the unwrapped points
 m_points_unwrapped.resize(m_points_wrapped.size());
 for (size_t i = 0; i < m_points_unwrapped.size(); i++) {
   m_points_unwrapped[i].resize(3);
 }

 for (size_t clust = 0; clust < cluster_count; clust++) {
   int idx=0;

   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
   kdtree.setInputCloud (clustered_clouds[clust]);

   for (size_t i = 0; i < clustered_clouds[clust]->points.size(); i++) {

     //we have a point and an index to insert it to, move the index until it winds the fisrt label
    //  while(labels.at<int>(idx,0)!=clust){
    //    idx++;
    //  }

    idx= clustered_points_indices[clust][i];

     m_points_unwrapped[idx][0]= clustered_clouds[clust]->points[i].x;
     m_points_unwrapped[idx][1]= clustered_clouds[clust]->points[i].y;
     m_points_unwrapped[idx][2]= clustered_clouds[clust]->points[i].z;


     //Need to check the nearest neightbours to fix the y (ditance to the plane)
     pcl::PointXYZ searchPoint;
     searchPoint=clustered_clouds[clust]->points[i];


     if (m_deform_walls){
       int K = 50;
       // double radius=0.1;
       double radius=0.05;
       //  double radius=0.16;

       std::vector<int> pointIdxNKNSearch(K);
       std::vector<float> pointNKNSquaredDistance(K);

       double avg_dist=0.0;

       if (  kdtree.radiusSearch (searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
       {
        for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p){
           // //average the dist of the K nearest neghbours
           avg_dist+=clustered_clouds[clust]->points[ pointIdxNKNSearch[p] ].y;
         }
       }

       avg_dist=avg_dist/pointIdxNKNSearch.size ();
       m_points_unwrapped[idx][1]=m_points_unwrapped[idx][1]-avg_dist;
     }




     //clap the distance
     // if ( fabs(m_points_unwrapped[idx][1]) > 0.1 ){
     //   m_points_unwrapped[idx][1]=0.0;
     //   clustered_clouds[clust]->points[i].y=0.0;
     // }

   }
 }







}


//unwrapp that assing each points to a cluster not based on normals but based on distance to edges
// void Model::compute_unwrap4(){
//   std::cout << "computing unwrap 2" << std::endl;
//   int cluster_count = m_num_walls;
//
//   //vector of planes that will be ftted to the clouds
//   planes.clear();
//   planes.resize(m_num_walls);
//   for (size_t i = 0; i < m_num_walls; i++) {
//      planes[i].coef.values.resize(4);
//      planes[i].index_cluster=i;
//    }
//
//
//   //Get the rough planes
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
//   seg.setOptimizeCoefficients (false);
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (400);
//   // seg.setDistanceThreshold (0.06);
//   seg.setDistanceThreshold (0.04);
//   // seg.setDistanceThreshold (0.10);
//
//   int i=0, nr_points = (int) m_points_wrapped_ds->points.size ();
//
//   while (i<m_num_walls)
//   {
//     // Segment the largest planar component from the remaining cloud
//     seg.setInputCloud (m_points_wrapped_ds);
//     // seg.segment (*inliers, *coefficients);
//     seg.segment (*inliers, (planes[i].coef));
//     if (inliers->indices.size () == 0)
//     {
//       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//       break;
//     }
//
//     // Extract the planar inliers from the input cloud
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud (m_points_wrapped_ds);
//     extract.setIndices (inliers);
//     extract.setNegative (false);
//
//     // Get the points associated with the planar surface
//     extract.filter (*cloud_plane);
//     std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//
//     // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
//     // viewer.showCloud (cloud_plane);
//     // while (!viewer.wasStopped ())
//     // {
//     // }
//
//     // Remove the planar inliers, extract the rest
//     extract.setNegative (true);
//     extract.filter (*cloud_f);
//     *m_points_wrapped_ds = *cloud_f;
//
//     i++;
//   }
//
//
//
//   //Add the normal as an eigen vector for easier calculations
//   for (size_t i = 0; i < planes.size(); i++) {
//     planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
//   }
//
//
//   //Fix normals of planes so that they point towards the center
//   //Dot product between the (point on a plane- center) and the normal (if it's negative then flip the normal)
//   //WATCHOUT I SET Z and Y to 0 because if I set x and y Imight not get a solution since the plane might be paralel to Z axis.
//   //Needs a more reobust way of finding points to account for parallel planes
//   for (size_t i = 0; i < planes.size(); i++) {
//     Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ) - m_center[0],
//                         - m_center[1],
//                         - m_center[2]);
//
//     double dot= planes[i].normal.dot(dir);
//
//     if (dot<0.0){
//       std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
//       planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
//     }
//   }
//
//
//
//   //get the angles of the planes with respect to the x axis
//   for (size_t i = 0; i < planes.size(); i++) {
//       Eigen::Vector2f normal = Eigen::Vector2f::Map(planes[i].coef.values.data(), 2);
//       Eigen::Vector2f x_axis (1.0, 0.0);
//
//
//       float dot = normal.dot(x_axis);      //dot produt
//       float cross = normal.x()*x_axis.y() - x_axis.x()*normal.y(); //cross, determinant
//       double angle = atan2(cross, dot) ;  // atan2(y, x) or atan2(sin, cos)
//
//       angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);
//
//       planes[i].angle=angle;
//   }
//   std::sort(planes.begin(), planes.end(), by_angle());
//
//
//
//
//
//   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//   //
//   // cloud->width    = m_num_walls;
//   // cloud->height   = 1;
//   // cloud->is_dense = false;
//   // cloud->points.resize (m_num_walls);
//
//
//   std::vector <point_struct> corners_mesh(m_num_walls);
//   //go though every plane pair and get the line between them
//   //https://www.mathworks.com/matlabcentral/fileexchange/17618-plane-intersection/content/plane_intersect.m
//   std::vector <line_struct> lines(planes.size());
//      for (size_t i = 0; i < planes.size(); i++) {
//        int curr = i;
//        int next = i +1;
//        if (next >= planes.size()) {
//          next=0;
//        }
//
//        std::cout << "intersect betwen " << curr << " " << next  << std::endl;
//        //need the direction vetor which is the cross product between the normals of the two planes_ordered
//        row_type N_1(3);
//        row_type N_2(3);
//
//        N_1[0]=planes[curr].coef.values[0];
//        N_1[1]=planes[curr].coef.values[1];
//        N_1[2]=planes[curr].coef.values[2];
//
//        N_2[0]=planes[next].coef.values[0];
//        N_2[1]=planes[next].coef.values[1];
//        N_2[2]=planes[next].coef.values[2];
//
//        row_type A1(3);  //Point that belong on plane 1
//        row_type A2(3);  //Point that belong on plane 2
//
//        A1[0]=(- (planes[curr].coef.values[3] / planes[curr].coef.values[0]) );
//        A1[1]=0.0;
//        A1[2]=0.0;
//
//        A2[0]=(- (planes[next].coef.values[3] / planes[next].coef.values[0]) );
//        A2[1]=0.0;
//        A2[2]=0.0;
//
//
//        row_type P(3);  //Point on the line that lies on the intersection
//        row_type N(3);  //direction vector of the intersect line
//
//        //  N=cross(N1,N2);
//        N[0]= N_1[1]*N_2[2] - N_1[2]*N_2[1];
//        N[1]= N_1[2]*N_2[0] - N_1[0]*N_2[2];
//        N[2]= N_1[0]*N_2[1] - N_1[1]*N_2[0];
//
//        std::cout << "N is " << N[0] << " " << N[1] << " " << N[2] << std::endl;
//
//        double d1=planes[curr].coef.values[3]; //constant of plane 1
//        double d2=planes[next].coef.values[3]; //constant of plane 2
//
//
//        //find the maximum coordinate in the N vector and 0 that one
//        int maxc=std::distance(N.begin(), std::max_element(N.begin(), N.end(), abs_compare));
//        std::cout << "maxc is" << maxc << std::endl;
//
//        //TODO:Fill the other cases.
//        switch (maxc) {
//          case 0:
//
//          break;
//
//          case 1:
//          break;
//
//          case 2:
//
//          break;
//        }
//
//        P[0]=(d2*N_1[1] - d1*N_2[1]) / N[2];
//        P[1]=(d1*N_2[0] - d2*N_1[0]) / N[2];
//        P[2]=0.0;
//
//        lines[i].direction=N;
//        lines[i].point=P;
//
//        std::cout << "point of intersect is" << P[0] << " " << P[1] << " " << P[2] << std::endl;
//
//        corners_mesh[i].point=P;
//        corners_mesh[i].index=i;
//
//       //  draw_sphere(P[0], P[1], P[2] );
//       //
//       // cloud->points[i].x =  P[0];
//       // cloud->points[i].y =  P[1];
//       // cloud->points[i].z =  P[2];
//       //
//       // // if(i==6){
//       //   cloud->points[i].r=255;
//       //   cloud->points[i].g=0;
//       //   cloud->points[i].b=0;
//       // // }else{
//       // //   cloud->points[i].r=255;
//       // //   cloud->points[i].g=255;
//       // //   cloud->points[i].b=255;
//       // // }
//
//
//      }
//
//
//   //Show intersection points
//   // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
//   // viewer.showCloud (cloud);
//   // while (!viewer.wasStopped ())
//   // {
//   // }
//
//
//   // m_points_wrapped_ds=compute_decimated(m_points_wrapped);
//   // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   // viewer->setBackgroundColor (0, 0, 0);
//   // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
//   // viewer->addPointCloud<pcl::PointXYZ> (m_points_wrapped_ds, "sample cloud2");
//   // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   // viewer->addCoordinateSystem (1.0);
//   // viewer->initCameraParameters ();
//   // while (!viewer->wasStopped ())
//   // {
//   //   viewer->spinOnce (100);
//   //   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//   // }
//
//
//
//
//
//   std::cout << "finished getting the corner points" << std::endl;
//
//
//   matrix_type_i clustered_points_indices(m_num_walls);
//
//
//   //Build a vector of cloud points
//   //vector of clouds
//   clustered_clouds.clear();
//   for (size_t clust = 0; clust < m_num_walls; clust++) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    clustered_clouds.push_back(cloud);
//   }
//
//
//   //For each point in the mesh order the corner of the mesh by distance
//   for (size_t i = 0; i < m_points_wrapped.size(); i++) {
//
//     //Set the distance from that points to all corner of the mesh
//     for (size_t corner_idx = 0; corner_idx < m_num_walls; corner_idx++) {
//       double distance=0.0;
//       distance= dist_no_z(m_points_wrapped[i], corners_mesh[corner_idx].point);
//       corners_mesh[corner_idx].distance=distance;
//     }
//
//     //order them by distance
//     std::sort(corners_mesh.begin(), corners_mesh.end(), by_distance());
//
//     //grab the 2 first points, the max of heir index-1 is the cluster that the points is assigned to
//
//     int second_pt_idx=1;
//     //if the angle between the 3 of them is too far away from the ideal 180, grab as the second points the third one
//     double angle;
//     angle=find_angle(corners_mesh[0].point,m_points_wrapped[i],corners_mesh[1].point);
//     angle=angle*180.0/M_PI;
//     // std::cout << "angle is: " << angle << std::endl;
//     angle=std::fabs(angle-180.0);
//     // if (angle>60.0){ //deviation from the 180
//     //   // second_pt_idx=2;
//     //
//     //   int idx_0=corners_mesh[0].index;
//     //   int idx_1=corners_mesh[1].index;
//     //   int idx_2=corners_mesh[2].index;
//     //
//     //   if (idx_1>idx_0) idx_2=idx_0-1;
//     //   if (idx_1<idx_0) idx_2=idx_0+1;
//     //
//     //   //special cases
//     //   if (idx_0==0 && idx_1==m_num_walls-1)  idx_2=1;
//     //   if (idx_0==0 && idx_1==1)  idx_2=m_num_walls-1;
//     //   if (idx_0==m_num_walls-1 && idx_1==m_num_walls-2)  idx_2=0;
//     //   if (idx_0==m_num_walls-1 && idx_1==0)  idx_2=m_num_walls-2;
//     //
//     //   //find the corner that has index equals to idx2
//     //   for (size_t cor = 0; cor < corners_mesh.size(); cor++) {
//     //     if(corners_mesh[cor].index==idx_2){
//     //         second_pt_idx=cor;
//     //     }
//     //   }
//     //
//     // }
//
//     //second attempt look at the second posibility of angle and check if its better
//     int idx_0=corners_mesh[0].index;
//     int idx_1=corners_mesh[1].index;
//     int idx_2;
//
//     if (idx_1>idx_0) idx_2=idx_0-1;
//     if (idx_1<idx_0) idx_2=idx_0+1;
//
//     //special cases
//     if (idx_0==0 && idx_1==m_num_walls-1)  idx_2=1;
//     if (idx_0==0 && idx_1==1)  idx_2=m_num_walls-1;
//     if (idx_0==m_num_walls-1 && idx_1==m_num_walls-2)  idx_2=0;
//     if (idx_0==m_num_walls-1 && idx_1==0)  idx_2=m_num_walls-2;
//
//     // std::cout << "idexes are: "<< idx_0 << " " << idx_1 << " : " << idx_2  << std::endl;
//
//     //find the corner that has index equals to idx2
//     for (size_t cor = 0; cor < corners_mesh.size(); cor++) {
//       if(corners_mesh[cor].index==idx_2){
//           second_pt_idx=cor;
//           break;
//       }
//     }
//
//     //check if its grabbing the points in the other direction is bettter (have lower deviation from 180)
//     double angle2;
//     angle2=find_angle(corners_mesh[0].point,m_points_wrapped[i],corners_mesh[second_pt_idx].point);
//     angle2=angle2*180.0/M_PI;
//     angle2=std::fabs(angle2-180.0);
//
//     if (angle2>angle){
//       second_pt_idx=1;
//     }else{
//       //angle2  is better so we assing the point to the other cluster
//       // std::cout << "closest points are: " << corners_mesh[0].index << " and " << corners_mesh[1].index << std::endl;
//       // std::cout << "angle is: " <<angle << std::endl;
//       // std::cout << "angle2 is: " << angle2 << std::endl;
//
//       // if (angle2>70){
//       //   std::cout << "closest points are: " << corners_mesh[0].index << " and " << corners_mesh[1].index << std::endl;
//       //   std::cout << "angle is: " <<angle << std::endl;
//       //   std::cout << "angle2 is: " << angle2 << std::endl;
//       //   std::cin.get();
//       // }
//
//     }
//
//     // if (angle2>70){
//     //   std::cout << "angle2 is: " <<angle2 << std::endl;
//     //    std::cin.get();
//     // }
//     // if (angle>70){
//     //   std::cout << "angle is: "<< angle << std::endl;
//     //   std::cin.get();
//     // }
//
//
//
//
//
//
//     int clust_idx= std::max(corners_mesh[0].index, corners_mesh[second_pt_idx].index) -1 ;
//     int min_idx=std::min(corners_mesh[0].index, corners_mesh[second_pt_idx].index);
//     int max_idx=std::max(corners_mesh[0].index, corners_mesh[second_pt_idx].index);
//     if(min_idx==0 && max_idx==m_num_walls-1){
//       clust_idx=m_num_walls-1;
//     }
//
//
//     //With angles
//     // int idx_1=corners_mesh[0].index;
//     //
//     // //we the points of minimum distance
//     // //calculate the angle formed by our mesh point, min dist point and all the others
//     //
//     // corners_mesh[0].angle=999999; //set it to big value so that it goes to the finale when sorted by angle
//     //
//     // for (size_t corner_idx = 1; corner_idx < m_num_walls; corner_idx++) {
//     //   double angle=0.0;
//     //
//     //
//     //
//     //
//     //   // angle = std::acos((  std::pow(dist_no_z( m_points_wrapped[i],corners_mesh[0].point),2)  +
//     //   //                        std::pow(dist_no_z( m_points_wrapped[i],corners_mesh[corner_idx].point),2) -
//     //   //                        std::pow(dist_no_z(corners_mesh[0].point,corners_mesh[corner_idx].point),2)) /
//     //   //                        (2 * dist_no_z( m_points_wrapped[i],corners_mesh[0].point) *
//     //   //                             dist_no_z( m_points_wrapped[i],corners_mesh[corner_idx].point)));
//     //
//     //   angle=find_angle(corners_mesh[0].point,
//     //                    m_points_wrapped[i],
//     //                    corners_mesh[corner_idx].point);
//     //
//     //   angle=angle*180/M_PI;
//     //   //We need it to be close to 180
//     //   angle=std::fabs(angle-180.0);
//     //
//     //   // std::cout << "angle is: " << angle << std::endl;
//     //   corners_mesh[corner_idx].angle=angle;
//     // }
//     // std::sort(corners_mesh.begin(), corners_mesh.end(), by_angle());
//     //
//     // int idx_2=corners_mesh[0].index;
//     //
//     //
//     //
//     // int clust_idx= std::max(idx_1, idx_2) -1 ;
//     //
//     // if(idx_1==0 && idx_2==m_num_walls-1){
//     //   clust_idx=m_num_walls-1;
//     // }
//     // if(idx_2==0 && idx_1==m_num_walls-1){
//     //   clust_idx=m_num_walls-1;
//     // }
//
//
//
//
//     // std::cout << "clust_idx assigned is: " << clust_idx << std::endl;
//
//     pcl::PointXYZ pt;
//     pt.x=m_points_wrapped[i][0];
//     pt.y=m_points_wrapped[i][1];
//     pt.z=m_points_wrapped[i][2];
//
//
//     clustered_clouds[clust_idx]->points.push_back(pt);
//
//     clustered_points_indices[clust_idx].push_back(i);
//   }
//
//   std::cout << "finished clutering points" << std::endl;
//
//   for (size_t i = 0; i < m_num_walls; i++) {
//     // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
//     // viewer.showCloud (clustered_clouds[i]);
//     // while (!viewer.wasStopped ())
//     // {
//     // }
//   }
//
//
//   //-----------------------
//
//
//
//
//   //NOW we have the clutered clouds
//
//
//
//
//   //Decimate the clouds
//   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
//   for (size_t clust = 0; clust < cluster_count; clust++) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     clustered_clouds_ds.push_back(cloud);
//   }
//   for (size_t clust = 0; clust < cluster_count; clust++) {
//     pcl::VoxelGrid<pcl::PointXYZ> ds;  //create downsampling filter
//     ds.setInputCloud (clustered_clouds[clust]);
//     ds.setLeafSize (0.1, 0.1, 0.1);
//     ds.filter (*clustered_clouds_ds[clust]);
//   }
//
//
//
//
//   //vector of planes that will be ftted to the clouds
//   planes.clear();
//   planes.resize(cluster_count);
//   for (size_t i = 0; i < cluster_count; i++) {
//      planes[i].coef.values.resize(4);
//      planes[i].index_cluster=i;
//    }
//
//
//   std::cout << "segmenting" << std::endl;
//
//   m_inliers_vec.clear();
//
//   #pragma omp parallel for
//   for (size_t clust = 0; clust < cluster_count; clust++) {
//     pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
//     pcl::SACSegmentation<pcl::PointXYZ> seg2;
//
//     //  seg.setOptimizeCoefficients (true);
//
//     seg2.setModelType (pcl::SACMODEL_PLANE);
//     seg2.setMethodType (pcl::SAC_RANSAC);
//     //  seg.setDistanceThreshold (0.02);
//     seg2.setDistanceThreshold (0.05);
//    // seg.setDistanceThreshold (0.007);
//
//     seg2.setInputCloud (clustered_clouds[clust]);
//     seg2.segment (*inliers2, (planes[clust].coef));
//
//     if (inliers2->indices.size () == 0)
//     {
//       PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//     }
//
//     m_inliers_vec.push_back(inliers2);
//
//
//     std::cerr << "Model coefficients: " << planes[clust].coef.values[0] << " "
//                                          << planes[clust].coef.values[1] << " "
//                                          << planes[clust].coef.values[2] << " "
//                                          << planes[clust].coef.values[3] << std::endl;
//  }
//
//
//  //Add the normal as an eigen vector for easier calculations
//  for (size_t i = 0; i < planes.size(); i++) {
//    planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
//  }
//
//
//  //Fix normals of planes so that they point towards the center
//  //Dot product between the (point on a plane- center) and the normal (if it's negative then flip the normal)
//  //WATCHOUT I SET Z and Y to 0 because if I set x and y Imight not get a solution since the plane might be paralel to Z axis.
//  //Needs a more reobust way of finding points to account for parallel planes
//  for (size_t i = 0; i < planes.size(); i++) {
//    Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ) - m_center[0],
//                        - m_center[1],
//                        - m_center[2]);
//
//    double dot= planes[i].normal.dot(dir);
//
//    if (dot<0.0){
//      std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
//      planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
//    }
//  }
//
//
//
//  //get the angles of the planes with respect to the x axis
//  for (size_t i = 0; i < planes.size(); i++) {
//      Eigen::Vector2f normal = Eigen::Vector2f::Map(planes[i].coef.values.data(), 2);
//      Eigen::Vector2f x_axis (1.0, 0.0);
//
//
//      float dot = normal.dot(x_axis);      //dot produt
//      float cross = normal.x()*x_axis.y() - x_axis.x()*normal.y(); //cross, determinant
//      double angle = atan2(cross, dot) ;  // atan2(y, x) or atan2(sin, cos)
//
//      angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);
//
//      planes[i].angle=angle;
//  }
//  std::sort(planes.begin(), planes.end(), by_angle());
//
//
//
//
//  //Writing the original walls in the model
//  // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
//  clustered_clouds_original.clear();
//  for (size_t clust = 0; clust < cluster_count; clust++) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    clustered_clouds_original.push_back(cloud);
//  }
//  for (size_t clust = 0; clust < cluster_count; clust++) {
//    clustered_clouds_original[clust]->width    = clustered_clouds[clust]->points.size();
//    clustered_clouds_original[clust]->height   = 1;
//    clustered_clouds_original[clust]->is_dense = false;
//    clustered_clouds_original[clust]->points.resize (clustered_clouds[clust]->points.size());
//
//    for (size_t i = 0; i < clustered_clouds[clust]->points.size(); i++){
//      clustered_clouds_original[clust]->points[i].x =  clustered_clouds[clust]->points[i].x;
//      clustered_clouds_original[clust]->points[i].y =  clustered_clouds[clust]->points[i].y;
//      clustered_clouds_original[clust]->points[i].z =  clustered_clouds[clust]->points[i].z;
//    }
//
//  }
//
//
//
//
//
//
//
//
//
//  // go through the planes and calculate the rotation translation matrix to map them
//  m_plane_centers.resize(planes.size());
//  #pragma omp parallel for
//  for (size_t i = 0; i < planes.size(); i++) {
//    //TODO::MAKE ALL this code use to use eigen, clean it
//
//    vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
//    planeSource->SetCenter(0.0, 0.0, 0.0);
//    planeSource->SetNormal( planes[i].coef.values[0],
//                            planes[i].coef.values[1],
//                            planes[i].coef.values[2]);
//    planeSource->Push(-planes[i].coef.values[3]);
//    planeSource->Update();
//
//    double center_vtk[3];
//    planeSource->GetCenter(center_vtk);
//    m_plane_centers[i]=row_type {center_vtk[0],center_vtk[1],center_vtk[2]};
//    // Eigen::Vector3f plane_center = Eigen::Vector3f::Map(center_vtk, 3);
//    // plane_center[2]=0.0;
//
//    Eigen::Vector3f plane_center;
//    plane_center[0]=center_vtk[0];
//    plane_center[1]=center_vtk[1];
//    plane_center[2]=0.0;
//
//
//    // double offset= 0.03*i;  //TODO REMOVE: Each plane gets closer and closer
//    // double offset= 0.05*i;  //TODO REMOVE: Each plane gets closer and closer
//    double offset= 0.00*i;  //TODO REMOVE: Each plane gets closer and closer
//
//
//    row_type end_point(3);
//    end_point[0]= planes[i].angle * m_circumference - offset; //I don't know why it needs to be summed and not -
//    end_point[1]= 0.0;
//    end_point[2]= 0.0;
//
//
//    //TEST: Second way of calculating the end point
//    // double angle_wall=360.0/(double)m_num_walls;
//    // angle_wall= angle_wall *M_PI/180.0;
//    // double side= sqrt (m_radius*m_radius +m_radius*m_radius -2*m_radius*m_radius* cos (angle_wall));
//    // // pos=pos/2;
//    // double pos= side*i;
//    // pos=pos- (side/2);
//    // end_point[0]=pos;
//    //-----------------
//
//    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//    Eigen::Affine3f transform_center = Eigen::Affine3f::Identity();
//
//    // Define a translation of 2.5 meters on the x axis.
//    transform_2.translation() << end_point[0], end_point[1], end_point[2];
//    transform_center.translation() << -plane_center[0], -plane_center[1], -plane_center[2];
//
//
//    double r = (90 * M_PI /180) + interpolate (planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
//    transform_2.rotate (Eigen::AngleAxisf (r, Eigen::Vector3f::UnitZ()));
//
//
//    std::cout << transform_2.matrix() << std::endl;
//    int clust_idx= planes[i].index_cluster;
//
//    pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform_center);
//    pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform_2);
//
//  }
//
//
//
//
//  //Move the walls closer together
//  matrix_type min_max(cluster_count);
//  // for (size_t clust = 0; clust < cluster_count; clust++) {
//  //   //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
//  //   double min_x_cur=std::numeric_limits<double>::max();;
//  //   double max_x_cur=std::numeric_limits<double>::min();;
//  //   for (size_t i_idx = 0; i_idx < m_inliers_vec[clust]->indices.size (); i_idx++) {
//  //     if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x > max_x_cur  ){
//  //       max_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
//  //     }
//  //     if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x < min_x_cur  ){
//  //       min_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
//  //     }
//  //
//  //   }
//  //   // std::cout << "cluster " << clust  << std::endl;
//  //   // std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;
//  //
//  //   min_max[clust]=row_type {min_x_cur, max_x_cur};  //there are now unordered
//  // }
//
//
//  for (size_t clust = 0; clust < cluster_count; clust++) {
//    //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
//    double min_x_cur=std::numeric_limits<double>::max();;
//    double max_x_cur=std::numeric_limits<double>::min();;
//    for (size_t i_idx = 0; i_idx < clustered_clouds[clust]->points.size (); i_idx++) {
//      if ( clustered_clouds[clust]->points[i_idx].x > max_x_cur  ){
//        max_x_cur=clustered_clouds[clust]->points[i_idx].x;
//      }
//      if ( clustered_clouds[clust]->points[i_idx].x < min_x_cur  ){
//        min_x_cur=clustered_clouds[clust]->points[i_idx].x;
//      }
//
//    }
//    // std::cout << "cluster " << clust  << std::endl;
//    // std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;
//
//    min_max[clust]=row_type {min_x_cur, max_x_cur};  //there are now unordered
//  }
//
//
//  //Now that we have the min and max, move them
//  std::cout << "movements" << std::endl;
//  row_type movements(cluster_count, 0.0);
//  for (size_t i = 0; i < planes.size(); i++) {
//    int clust_idx= planes[i].index_cluster;
//    if (i==0){
//      movements[clust_idx]= 0.0 - min_max[clust_idx][0];
//    }else{
//      movements[clust_idx]=0.0;
//      movements[clust_idx]+= movements[planes[i-1].index_cluster];
//      movements[clust_idx]+= (min_max[planes[i-1].index_cluster][1] - min_max[clust_idx][0] );
//    }
//    std::cout << "clust index " << clust_idx << std::endl;
//    std::cout << "movemnt: " << movements[clust_idx] << std::endl;
//  }
//
//  std::cout << "move them given the min and max" << std::endl;
//  for (size_t i = 0; i < planes.size(); i++) {
//    int clust_idx= planes[i].index_cluster;
//
//
//
//    std::cout << "plane: " << i << std::endl;
//    std::cout << "min max: " << min_max[clust_idx][0] << " " << min_max[clust_idx][1] << std::endl;
//
//
//
//    //Move it
//    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//    // transform.translation() << min_x_cur - max_x_prev, 0.0, 0.0;
//    // pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform);
//    for (size_t p_idx = 0; p_idx < clustered_clouds[clust_idx]->points.size(); p_idx++) {
//      clustered_clouds[clust_idx]->points[p_idx].x = clustered_clouds[clust_idx]->points[p_idx].x + movements[clust_idx];
//    }
//
//  }
//
//
//  //showing stuff
//  // for (size_t i = 0; i < planes.size(); i++) {
//  //   int clust_idx= planes[i].index_cluster;
//  //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
//  //    viewer.showCloud (clustered_clouds[clust_idx]);
//  //    while (!viewer.wasStopped ())
//  //    {
//  //    }
//  // }
//
//
//
//
//  std::cout << "wiritng to unwrapped points" << std::endl;
//
//  //write the unwrapped points
//  m_points_unwrapped.resize(m_points_wrapped.size());
//  for (size_t i = 0; i < m_points_unwrapped.size(); i++) {
//    m_points_unwrapped[i].resize(3);
//  }
//
//  for (size_t clust = 0; clust < cluster_count; clust++) {
//    int idx=0;
//
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    kdtree.setInputCloud (clustered_clouds[clust]);
//
//    for (size_t i = 0; i < clustered_clouds[clust]->points.size(); i++) {
//
//      //we have a point and an index to insert it to, move the index until it winds the fisrt label
//     //  while(labels.at<int>(idx,0)!=clust){
//     //    idx++;
//     //  }
//
//     idx= clustered_points_indices[clust][i];
//
//      m_points_unwrapped[idx][0]= clustered_clouds[clust]->points[i].x;
//      m_points_unwrapped[idx][1]= clustered_clouds[clust]->points[i].y;
//      m_points_unwrapped[idx][2]= clustered_clouds[clust]->points[i].z;
//
//
//      //Need to check the nearest neightbours to fix the y (ditance to the plane)
//      pcl::PointXYZ searchPoint;
//      searchPoint=clustered_clouds[clust]->points[i];
//
//
//      if (m_deform_walls){
//        int K = 50;
//        // double radius=0.1;
//        double radius=0.05;
//        //  double radius=0.16;
//
//        std::vector<int> pointIdxNKNSearch(K);
//        std::vector<float> pointNKNSquaredDistance(K);
//
//        double avg_dist=0.0;
//
//        if (  kdtree.radiusSearch (searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//        {
//         for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p){
//            // //average the dist of the K nearest neghbours
//            avg_dist+=clustered_clouds[clust]->points[ pointIdxNKNSearch[p] ].y;
//          }
//        }
//
//        avg_dist=avg_dist/pointIdxNKNSearch.size ();
//        m_points_unwrapped[idx][1]=m_points_unwrapped[idx][1]-avg_dist;
//      }
//
//
//
//
//      //clap the distance
//      // if ( fabs(m_points_unwrapped[idx][1]) > 0.1 ){
//      //   m_points_unwrapped[idx][1]=0.0;
//      //   clustered_clouds[clust]->points[i].y=0.0;
//      // }
//
//    }
//  }
//
//
//
//
//
//
//
//
//
//
//
//   // //Blur the normals so as to better detect the walls
//   // blur_normals();
//   //
//   // //Cluster the normals of the points.
//   // cv::Mat samples(m_normals_blured.size(), 3, CV_32F);
//   // for( int y = 0; y < samples.rows; y++ ){
//   //   for( int x = 0; x < samples.cols; x++ ){
//   //     samples.at<float>(y,x)=m_normals_blured[y][x];
//   //   }
//   // }
//   //
//   // // row_type angles_test= compute_angles(m_points_wrapped);
//   // // cv::Mat samples(m_normals.size(), 4, CV_32F);
//   // // for( int y = 0; y < samples.rows; y++ ){
//   // //   for( int x = 0; x < 3; x++ ){
//   // //     samples.at<float>(y,x)=m_normals[y][x];
//   // //   }
//   // //   for( int x = 3; x < 4; x++ ){
//   // //     // samples.at<float>(y,x)=m_points_wrapped[y][x-3];
//   // //     samples.at<float>(y,x)=angles_test[y];
//   // //   }
//   // // }
//   //
//   // int cluster_count = m_num_walls;
//   // cv::Mat labels;
//   // int attempts = 10;
//   // cv::Mat centers;
//   // cv::kmeans(samples, cluster_count, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.001), attempts, cv::KMEANS_PP_CENTERS, centers );
//   //
//   // //Get number of elements in each cluster
//   // std::vector<int> labels_vec;
//   // for (size_t i = 0; i < labels.rows; i++) {
//   //   labels_vec.push_back(labels.at<int>(i,0));
//   // }
//   //
//   //
//   //
//   // //Build a vector of cloud points and then fit planes through each cloud
//   // //vector of clouds
//   // clustered_clouds.clear();
//   // for (size_t clust = 0; clust < cluster_count; clust++) {
//   //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   //  clustered_clouds.push_back(cloud);
//   // }
//   //
//   //  for (size_t i = 0; i < m_points_wrapped.size(); i++){
//   //     int label=labels.at<int>(i,0);
//   //     pcl::PointXYZ p;
//   //     p.x=m_points_wrapped[i][0];
//   //     p.y=m_points_wrapped[i][1];
//   //     p.z=m_points_wrapped[i][2];
//   //     clustered_clouds[label]->push_back(p);
//   //  }
//   //
//   //
//   //
//   //  //Decimate the clouds
//   //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
//   //  for (size_t clust = 0; clust < cluster_count; clust++) {
//   //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   //    clustered_clouds_ds.push_back(cloud);
//   //  }
//   //  for (size_t clust = 0; clust < cluster_count; clust++) {
//   //    pcl::VoxelGrid<pcl::PointXYZ> ds;  //create downsampling filter
//   //    ds.setInputCloud (clustered_clouds[clust]);
//   //    ds.setLeafSize (0.1, 0.1, 0.1);
//   //    ds.filter (*clustered_clouds_ds[clust]);
//   //
//   //   //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
//   //   //  viewer.showCloud (clustered_clouds_ds[clust]);
//   //   //  while (!viewer.wasStopped ())
//   //   //  {
//   //   //  }
//   //  }
//   //
//   //
//   //
//   //
//   // //vector of planes that will be ftted to the clouds
//   // planes.clear();
//   // planes.resize(cluster_count);
//   // for (size_t i = 0; i < cluster_count; i++) {
//   //    planes[i].coef.values.resize(4);
//   //    planes[i].index_cluster=i;
//   //  }
//   //
//   //
//   //  std::cout << "segmenting" << std::endl;
//   //
//   //  m_inliers_vec.clear();
//   //
//   //  #pragma omp parallel for
//   //  for (size_t clust = 0; clust < cluster_count; clust++) {
//   //    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   //    pcl::SACSegmentation<pcl::PointXYZ> seg;
//   //
//   //    //  seg.setOptimizeCoefficients (true);
//   //
//   //    seg.setModelType (pcl::SACMODEL_PLANE);
//   //    seg.setMethodType (pcl::SAC_RANSAC);
//   //    //  seg.setDistanceThreshold (0.02);
//   //    seg.setDistanceThreshold (0.011);
//   //   // seg.setDistanceThreshold (0.007);
//   //
//   //    seg.setInputCloud (clustered_clouds[clust]);
//   //    seg.segment (*inliers, (planes[clust].coef));
//   //
//   //    if (inliers->indices.size () == 0)
//   //    {
//   //      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//   //    }
//   //
//   //    m_inliers_vec.push_back(inliers);
//   //
//   //
//   //    std::cerr << "Model coefficients: " << planes[clust].coef.values[0] << " "
//   //                                         << planes[clust].coef.values[1] << " "
//   //                                         << planes[clust].coef.values[2] << " "
//   //                                         << planes[clust].coef.values[3] << std::endl;
//   // }
//   //
//   //
//   // //Add the normal as an eigen vector for easier calculations
//   // for (size_t i = 0; i < planes.size(); i++) {
//   //   planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
//   // }
//   //
//   //
//   // //Fix normals of planes so that they point towards the center
//   // //Dot product between the (point on a plane- center) and the normal (if it's negative then flip the normal)
//   // //WATCHOUT I SET Z and Y to 0 because if I set x and y Imight not get a solution since the plane might be paralel to Z axis.
//   // //Needs a more reobust way of finding points to account for parallel planes
//   // for (size_t i = 0; i < planes.size(); i++) {
//   //   Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ) - m_center[0],
//   //                       - m_center[1],
//   //                       - m_center[2]);
//   //
//   //   double dot= planes[i].normal.dot(dir);
//   //
//   //   if (dot<0.0){
//   //     std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
//   //     planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
//   //   }
//   // }
//   //
//   //
//   //
//   // //get the angles of the planes with respect to the x axis
//   // for (size_t i = 0; i < planes.size(); i++) {
//   //     Eigen::Vector2f normal = Eigen::Vector2f::Map(planes[i].coef.values.data(), 2);
//   //     Eigen::Vector2f x_axis (1.0, 0.0);
//   //
//   //
//   //     float dot = normal.dot(x_axis);      //dot produt
//   //     float cross = normal.x()*x_axis.y() - x_axis.x()*normal.y(); //cross, determinant
//   //     double angle = atan2(cross, dot) ;  // atan2(y, x) or atan2(sin, cos)
//   //
//   //     angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);
//   //
//   //     planes[i].angle=angle;
//   // }
//   // std::sort(planes.begin(), planes.end(), by_angle());
//   //
//   //
//   //
//   //
//   // //-------------
//   //
//   //
//   //
//   //
//   //
//   //
//   // //go though every plane pair and get the line between them
//   // //https://www.mathworks.com/matlabcentral/fileexchange/17618-plane-intersection/content/plane_intersect.m
//   // std::vector <line_struct> lines(planes_ordered.size());
//   //    for (size_t i = 0; i < planes_ordered.size(); i++) {
//   //      int curr = i;
//   //      int next = i +1;
//   //      if (next >= planes_ordered.size()) {
//   //        next=0;
//   //      }
//   //
//   //      std::cout << "intersect betwen " << curr << " " << next  << std::endl;
//   //      //need the direction vetor which is the cross product between the normals of the two planes_ordered
//   //      row_type N_1(3);
//   //      row_type N_2(3);
//   //
//   //      N_1[0]=planes_ordered[curr].coef.values[0];
//   //      N_1[1]=planes_ordered[curr].coef.values[1];
//   //      N_1[2]=planes_ordered[curr].coef.values[2];
//   //
//   //      N_2[0]=planes_ordered[next].coef.values[0];
//   //      N_2[1]=planes_ordered[next].coef.values[1];
//   //      N_2[2]=planes_ordered[next].coef.values[2];
//   //
//   //      row_type A1(3);  //Point that belong on plane 1
//   //      row_type A2(3);  //Point that belong on plane 2
//   //
//   //      A1[0]=(- (planes[curr].values[3] / planes[curr].values[0]) );
//   //      A1[1]=0.0;
//   //      A1[2]=0.0;
//   //
//   //      A2[0]=(- (planes[next].values[3] / planes[next].values[0]) );
//   //      A2[1]=0.0;
//   //      A2[2]=0.0;
//   //
//   //
//   //      row_type P(3);  //Point on the line that lies on the intersection
//   //      row_type N(3);  //direction vector of the intersect line
//   //
//   //      //  N=cross(N1,N2);
//   //      N[0]= N_1[1]*N_2[2] - N_1[2]*N_2[1];
//   //      N[1]= N_1[2]*N_2[0] - N_1[0]*N_2[2];
//   //      N[2]= N_1[0]*N_2[1] - N_1[1]*N_2[0];
//   //
//   //      std::cout << "N is " << N[0] << " " << N[1] << " " << N[2] << std::endl;
//   //
//   //      double d1=planes_ordered[curr].coef.values[3]; //constant of plane 1
//   //      double d2=planes_ordered[next].coef.values[3]; //constant of plane 2
//   //
//   //
//   //      //find the maximum coordinate in the N vector and 0 that one
//   //      int maxc=std::distance(N.begin(), std::max_element(N.begin(), N.end(), abs_compare));
//   //      std::cout << "maxc is" << maxc << std::endl;
//   //
//   //      //TODO:Fill the other cases.
//   //      switch (maxc) {
//   //        case 0:
//   //
//   //        break;
//   //
//   //        case 1:
//   //        break;
//   //
//   //        case 2:
//   //
//   //        break;
//   //      }
//   //
//   //      P[0]=(d2*N_1[1] - d1*N_2[1]) / N[2];
//   //      P[1]=(d1*N_2[0] - d2*N_1[0]) / N[2];
//   //      P[2]=0.0;
//   //
//   //      lines[i].direction=N;
//   //      lines[i].point=P;
//   //
//   //      std::cout << "point of intersect is" << P[0] << " " << P[1] << " " << P[2] << std::endl;
//   //
//   //      draw_sphere(P[0], P[1], P[2] );
//   //
//   //
//   //    }
//
//
// }
//


void Model::compute_unwrap2(){
  std::cout << "computing unwrap 2" << std::endl;

  // //Show normals
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //
  //   cloud->width    = m_normals.size();
  //   cloud->height   = 1;
  //   cloud->is_dense = false;
  //   cloud->points.resize (m_normals.size());
  //
  //   for (size_t i = 0; i < cloud->points.size(); i++){
  //     cloud->points[i].x =  m_normals[i][0];
  //     cloud->points[i].y =  m_normals[i][1];
  //     cloud->points[i].z =  m_normals[i][2];
  //   }
  //
  //
  //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  //    viewer.showCloud (cloud);
  //    while (!viewer.wasStopped ())
  //    {
  //    }



  //Blur the normals so as to better detect the walls
  blur_normals();

  //Cluster the normals of the points.
  cv::Mat samples(m_normals_blured.size(), 3, CV_32F);
  for( int y = 0; y < samples.rows; y++ ){
    for( int x = 0; x < samples.cols; x++ ){
      samples.at<float>(y,x)=m_normals_blured[y][x];
    }
  }

  // row_type angles_test= compute_angles(m_points_wrapped);
  // cv::Mat samples(m_normals.size(), 4, CV_32F);
  // for( int y = 0; y < samples.rows; y++ ){
  //   for( int x = 0; x < 3; x++ ){
  //     samples.at<float>(y,x)=m_normals[y][x];
  //   }
  //   for( int x = 3; x < 4; x++ ){
  //     // samples.at<float>(y,x)=m_points_wrapped[y][x-3];
  //     samples.at<float>(y,x)=angles_test[y];
  //   }
  // }

  int cluster_count = m_num_walls;
  cv::Mat labels;
  int attempts = 10;
  cv::Mat centers;
  cv::kmeans(samples, cluster_count, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.001), attempts, cv::KMEANS_PP_CENTERS, centers );

  //Get number of elements in each cluster
  std::vector<int> labels_vec;
  for (size_t i = 0; i < labels.rows; i++) {
    labels_vec.push_back(labels.at<int>(i,0));
  }



  //Build a vector of cloud points and then fit planes through each cloud
  //vector of clouds
  clustered_clouds.clear();
  for (size_t clust = 0; clust < cluster_count; clust++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   clustered_clouds.push_back(cloud);
  }

   for (size_t i = 0; i < m_points_wrapped.size(); i++){
      int label=labels.at<int>(i,0);
      pcl::PointXYZ p;
      p.x=m_points_wrapped[i][0];
      p.y=m_points_wrapped[i][1];
      p.z=m_points_wrapped[i][2];
      clustered_clouds[label]->push_back(p);
   }



   //Decimate the clouds
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
   for (size_t clust = 0; clust < cluster_count; clust++) {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
     clustered_clouds_ds.push_back(cloud);
   }
   for (size_t clust = 0; clust < cluster_count; clust++) {
     pcl::VoxelGrid<pcl::PointXYZ> ds;  //create downsampling filter
     ds.setInputCloud (clustered_clouds[clust]);
     ds.setLeafSize (0.1, 0.1, 0.1);
     ds.filter (*clustered_clouds_ds[clust]);

    //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
    //  viewer.showCloud (clustered_clouds_ds[clust]);
    //  while (!viewer.wasStopped ())
    //  {
    //  }
   }




  //vector of planes that will be ftted to the clouds
  planes.clear();
  planes.resize(cluster_count);
  for (size_t i = 0; i < cluster_count; i++) {
     planes[i].coef.values.resize(4);
     planes[i].index_cluster=i;
   }


   std::cout << "segmenting" << std::endl;

   m_inliers_vec.clear();

   #pragma omp parallel for
   for (size_t clust = 0; clust < cluster_count; clust++) {
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     pcl::SACSegmentation<pcl::PointXYZ> seg;

     //  seg.setOptimizeCoefficients (true);

     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     //  seg.setDistanceThreshold (0.02);
     seg.setDistanceThreshold (0.011);
    // seg.setDistanceThreshold (0.007);

     seg.setInputCloud (clustered_clouds[clust]);
     seg.segment (*inliers, (planes[clust].coef));

     if (inliers->indices.size () == 0)
     {
       PCL_ERROR ("Could not estimate a planar model for the given dataset.");
     }

     m_inliers_vec.push_back(inliers);


     std::cerr << "Model coefficients: " << planes[clust].coef.values[0] << " "
                                          << planes[clust].coef.values[1] << " "
                                          << planes[clust].coef.values[2] << " "
                                          << planes[clust].coef.values[3] << std::endl;
  }


  //Add the normal as an eigen vector for easier calculations
  for (size_t i = 0; i < planes.size(); i++) {
    planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
  }


  //Fix normals of planes so that they point towards the center
  //Dot product between the (point on a plane- center) and the normal (if it's negative then flip the normal)
  //WATCHOUT I SET Z and Y to 0 because if I set x and y Imight not get a solution since the plane might be paralel to Z axis.
  //Needs a more reobust way of finding points to account for parallel planes
  for (size_t i = 0; i < planes.size(); i++) {
    Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ) - m_center[0],
                        - m_center[1],
                        - m_center[2]);

    double dot= planes[i].normal.dot(dir);

    if (dot<0.0){
      std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
      planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
    }
  }



  //get the angles of the planes with respect to the x axis
  for (size_t i = 0; i < planes.size(); i++) {
      Eigen::Vector2f normal = Eigen::Vector2f::Map(planes[i].coef.values.data(), 2);
      Eigen::Vector2f x_axis (1.0, 0.0);


      float dot = normal.dot(x_axis);      //dot produt
      float cross = normal.x()*x_axis.y() - x_axis.x()*normal.y(); //cross, determinant
      double angle = atan2(cross, dot) ;  // atan2(y, x) or atan2(sin, cos)

      angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);

      planes[i].angle=angle;
  }
  std::sort(planes.begin(), planes.end(), by_angle());




  //Writing the original walls in the model
  // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
  clustered_clouds_original.clear();
  for (size_t clust = 0; clust < cluster_count; clust++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    clustered_clouds_original.push_back(cloud);
  }
  for (size_t clust = 0; clust < cluster_count; clust++) {
    clustered_clouds_original[clust]->width    = clustered_clouds[clust]->points.size();
    clustered_clouds_original[clust]->height   = 1;
    clustered_clouds_original[clust]->is_dense = false;
    clustered_clouds_original[clust]->points.resize (clustered_clouds[clust]->points.size());

    for (size_t i = 0; i < clustered_clouds[clust]->points.size(); i++){
      clustered_clouds_original[clust]->points[i].x =  clustered_clouds[clust]->points[i].x;
      clustered_clouds_original[clust]->points[i].y =  clustered_clouds[clust]->points[i].y;
      clustered_clouds_original[clust]->points[i].z =  clustered_clouds[clust]->points[i].z;
    }

  }









  // go through the planes and calculate the rotation translation matrix to map them
  m_plane_centers.resize(planes.size());
  #pragma omp parallel for
  for (size_t i = 0; i < planes.size(); i++) {
    //TODO::MAKE ALL this code use to use eigen, clean it

    vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetCenter(0.0, 0.0, 0.0);
    planeSource->SetNormal( planes[i].coef.values[0],
                            planes[i].coef.values[1],
                            planes[i].coef.values[2]);
    planeSource->Push(-planes[i].coef.values[3]);
    planeSource->Update();

    double center_vtk[3];
    planeSource->GetCenter(center_vtk);
    m_plane_centers[i]=row_type {center_vtk[0],center_vtk[1],center_vtk[2]};
    // Eigen::Vector3f plane_center = Eigen::Vector3f::Map(center_vtk, 3);
    // plane_center[2]=0.0;

    Eigen::Vector3f plane_center;
    plane_center[0]=center_vtk[0];
    plane_center[1]=center_vtk[1];
    plane_center[2]=0.0;


    // double offset= 0.03*i;  //TODO REMOVE: Each plane gets closer and closer
    // double offset= 0.05*i;  //TODO REMOVE: Each plane gets closer and closer
    double offset= 0.00*i;  //TODO REMOVE: Each plane gets closer and closer


    row_type end_point(3);
    end_point[0]= planes[i].angle * m_circumference - offset; //I don't know why it needs to be summed and not -
    end_point[1]= 0.0;
    end_point[2]= 0.0;


    //TEST: Second way of calculating the end point
    // double angle_wall=360.0/(double)m_num_walls;
    // angle_wall= angle_wall *M_PI/180.0;
    // double side= sqrt (m_radius*m_radius +m_radius*m_radius -2*m_radius*m_radius* cos (angle_wall));
    // // pos=pos/2;
    // double pos= side*i;
    // pos=pos- (side/2);
    // end_point[0]=pos;
    //-----------------

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_center = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << end_point[0], end_point[1], end_point[2];
    transform_center.translation() << -plane_center[0], -plane_center[1], -plane_center[2];


    double r = (90 * M_PI /180) + interpolate (planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
    transform_2.rotate (Eigen::AngleAxisf (r, Eigen::Vector3f::UnitZ()));


    std::cout << transform_2.matrix() << std::endl;
    int clust_idx= planes[i].index_cluster;

    pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform_center);
    pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform_2);

  }




  //Move the walls closer together
  matrix_type min_max(cluster_count);
  for (size_t clust = 0; clust < cluster_count; clust++) {
    //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
    double min_x_cur=std::numeric_limits<double>::max();;
    double max_x_cur=std::numeric_limits<double>::min();;
    for (size_t i_idx = 0; i_idx < m_inliers_vec[clust]->indices.size (); i_idx++) {
      if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x > max_x_cur  ){
        max_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
      }
      if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x < min_x_cur  ){
        min_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
      }

    }
    // std::cout << "cluster " << clust  << std::endl;
    // std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;

    min_max[clust]=row_type {min_x_cur, max_x_cur};  //there are now unordered
  }


  //Now that we have the min and max, move them
  std::cout << "movements" << std::endl;
  row_type movements(cluster_count, 0.0);
  for (size_t i = 0; i < planes.size(); i++) {
    int clust_idx= planes[i].index_cluster;
    if (i==0){
      movements[clust_idx]= 0.0 - min_max[clust_idx][0];
    }else{
      movements[clust_idx]=0.0;
      movements[clust_idx]+= movements[planes[i-1].index_cluster];
      movements[clust_idx]+= (min_max[planes[i-1].index_cluster][1] - min_max[clust_idx][0] );
    }
    std::cout << "clust index " << clust_idx << std::endl;
    std::cout << "movemnt: " << movements[clust_idx] << std::endl;
  }

  std::cout << "move them given the min and max" << std::endl;
  for (size_t i = 0; i < planes.size(); i++) {
    int clust_idx= planes[i].index_cluster;



    std::cout << "plane: " << i << std::endl;
    std::cout << "min max: " << min_max[clust_idx][0] << " " << min_max[clust_idx][1] << std::endl;



    //Move it
    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // transform.translation() << min_x_cur - max_x_prev, 0.0, 0.0;
    // pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform);
    for (size_t p_idx = 0; p_idx < clustered_clouds[clust_idx]->points.size(); p_idx++) {
      clustered_clouds[clust_idx]->points[p_idx].x = clustered_clouds[clust_idx]->points[p_idx].x + movements[clust_idx];
    }

  }


  //showing stuff
  // for (size_t i = 0; i < planes.size(); i++) {
  //   int clust_idx= planes[i].index_cluster;
  //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  //    viewer.showCloud (clustered_clouds[clust_idx]);
  //    while (!viewer.wasStopped ())
  //    {
  //    }
  // }




  std::cout << "wiritng to unwrapped points" << std::endl;

  //write the unwrapped points
  m_points_unwrapped.resize(m_points_wrapped.size());
  for (size_t i = 0; i < m_points_unwrapped.size(); i++) {
    m_points_unwrapped[i].resize(3);
  }

  for (size_t clust = 0; clust < cluster_count; clust++) {
    int idx=0;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (clustered_clouds[clust]);

    for (size_t i = 0; i < clustered_clouds[clust]->points.size(); i++) {

      //we have a point and an index to insert it to, move the index until it winds the fisrt label
      while(labels.at<int>(idx,0)!=clust){
        idx++;
      }

      m_points_unwrapped[idx][0]= clustered_clouds[clust]->points[i].x;
      m_points_unwrapped[idx][1]= clustered_clouds[clust]->points[i].y;
      m_points_unwrapped[idx][2]= clustered_clouds[clust]->points[i].z;


      //Need to check the nearest neightbours to fix the y (ditance to the plane)
      pcl::PointXYZ searchPoint;
      searchPoint=clustered_clouds[clust]->points[i];


      if (m_deform_walls){
        int K = 50;
        // double radius=0.1;
        double radius=0.05;
        //  double radius=0.16;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        double avg_dist=0.0;

        if (  kdtree.radiusSearch (searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
         for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p){
            // //average the dist of the K nearest neghbours
            avg_dist+=clustered_clouds[clust]->points[ pointIdxNKNSearch[p] ].y;
          }
        }

        avg_dist=avg_dist/pointIdxNKNSearch.size ();
        m_points_unwrapped[idx][1]=m_points_unwrapped[idx][1]-avg_dist;
      }




      //clap the distance
      // if ( fabs(m_points_unwrapped[idx][1]) > 0.1 ){
      //   m_points_unwrapped[idx][1]=0.0;
      //   clustered_clouds[clust]->points[i].y=0.0;
      // }


      idx++;
    }
  }




}



void Model::compute_unwrap(){
  std::cout << "compute_unwrap" << std::endl;
  //Check if we have a m_wall
  //If we already have an unwrap for it, show it, otherwise, calculate it

  m_angles              =compute_angles(m_points_wrapped);
  distances_to_radius = compute_distances_to_radius(m_points_wrapped, m_radius);


  //assign the new points
  m_points_unwrapped.resize(m_points_wrapped.size());
  for (size_t i = 0; i < m_points_unwrapped.size(); i++) {
    m_points_unwrapped[i].resize(3);
    m_points_unwrapped[i][0]=m_angles[i] *m_circumference;
    m_points_unwrapped[i][1]=distances_to_radius[i];
    m_points_unwrapped[i][2]=m_points_wrapped[i][2];
  }
  std::cout << "finish asign points" << std::endl;

}


std::vector<double> Model::compute_angles(matrix_type points){


  //we again suppose that the center is already at 0,0,0
  std::vector<double> angles;
  angles.resize(points.size());

  for (size_t i = 0; i < angles.size(); i++) {
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
  std::vector<double> dist;
  dist.resize(points.size());

  //Calculate first the distance to center.
  for (size_t i = 0; i < dist.size(); i++) {
    dist[i]= sqrt( points[i][0]*points[i][0]  + points[i][1]*points[i][1] );
    dist[i]= dist[i]-radius;
  }

  return dist;
}



void Model::compute_plain_colors(){


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

void Model::compute_original_colors(){

  m_colors_active->Reset();
  m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active->SetNumberOfComponents(3);
  m_colors_active->SetName("rgb");


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

void Model::compute_bright_colors(){
  m_colors_active->Reset();
  m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active->SetNumberOfComponents(3);
  m_colors_active->SetName("rgb");


  int num_points;
  if (m_is_unwrapped){
    num_points=m_points_unwrapped.size();
  }else{
    num_points=m_points_wrapped.size();
  }

  int num_colors=m_colors_bright->GetNumberOfTuples();
  int min = std::min (num_points, num_colors);
  for (vtkIdType i = 0; i < min; i++) {
    m_colors_active->InsertNextTuple(m_colors_bright->GetTuple(i));
  }


}


void Model::compute_depth_colors(){



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
    compute_unwrap2(); //Compute unwrap to actually get unwrapped points
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


void Model::create_grid(){



  //Downsample the points
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud= compute_decimated(m_points_unwrapped);
  get_mesh(); //Do this so that we set the unwrap point to the mesh and therefore have correct bounds
  m_bounds=m_wall->GetBounds();

  //Attempt 2, faster since we only create the boundinx boxes and don't cut the mesh
  std::cout << "creating grid" << std::endl;



  //define the bounds of the box that will move around and cut. Size is 1x1m
  double box_size_x=1.0;
  double box_size_z=1.0;

  int grid_size_x=ceil( (fabs(m_bounds[1]) + fabs(m_bounds[0]) ) /box_size_x);     //  xmax/box_size_x
  int grid_size_z=ceil( (fabs(m_bounds[5]) + fabs(m_bounds[4]) ) /box_size_z);     //  zmax/box_size_z

  std::cout << "grid_size_x and z is " << grid_size_x << " " << grid_size_z << std::endl;

  m_grid.resize(grid_size_z*grid_size_x );

  double x_min=m_bounds[0];
  double z_min=m_bounds[4];
  for (size_t i = 0; i < grid_size_z; i++) {
    for (size_t j = 0; j < grid_size_x; j++) {
      row_type bounds(6, 0.0);

      //Fill these bounds
      bounds[0]=x_min;
      bounds[1]=x_min+box_size_x;

      bounds[4]=z_min;
      bounds[5]=z_min+box_size_z;

      //The Y max and min will be the one of the mesh
      bounds[2]=m_bounds[2];
      bounds[3]=m_bounds[3];

      m_grid[ i*grid_size_x  +j  ]=bounds;

      x_min+=box_size_x;
    }
    x_min=m_bounds[0];
    z_min+=box_size_z;
  }

  //Detect cells that have no points inside
  row_type_i cells_occupied(m_grid.size(), 0);
  for (size_t point_idx = 0; point_idx < cloud->points.size(); point_idx++) {
    for (size_t cell_idx = 0; cell_idx < m_grid.size(); cell_idx++) {
      bool ocupied=is_contained(cloud->points[point_idx], m_grid[cell_idx]);
      if (ocupied){
        cells_occupied[cell_idx]=1;
      }
    }
  }


  //delete the ones that have 0 in cells_occupied
  int i=0;
  while ( i < m_grid.size() ) {
    if ( cells_occupied[i]==0 ) {
        m_grid.erase(m_grid.begin() + i);
        cells_occupied.erase(cells_occupied.begin() + i);
    } else {
        ++i;
    }
  }


  m_grid_cells_active.resize(m_grid.size(), 0);
  std::cout << "model: finished creating the grid with" << m_grid.size() << std::endl;


}


bool Model::is_contained(pcl::PointXYZ point , row_type bounds){

  if (point.x < bounds[0] || point.x > bounds[1])  //x
    return false;
  // if (point.y < bounds[2] || point.y > bounds[3])  //y  //Do not chck in the y axis because it's so small
  //   return false;
  if (point.z < bounds[4] || point.z > bounds[5])  //z
    return false;

  return true;
}

bool Model::is_contained(row_type point , row_type bounds){

  if (point[0] < bounds[0] || point[0] > bounds[1])  //x
    return false;
  // if (point[1] < bounds[2] || point[1] > bounds[3])  //y   //Do not chck in the y axis because it's so small
  //   return false;
  if (point[2] < bounds[4] || point[2] > bounds[5])  //z
    return false;

  return true;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr Model::compute_decimated(matrix_type points){

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


void Model::right_click_pressed_slot(row_type point){
  std::cout << "Model:: received points" << point[0] << " " << point[1]  << " "<< point[2] << std::endl;
  for (size_t i = 0; i < m_grid.size(); i++) {
    bool occupied= is_contained(point, m_grid[i] );
    if (occupied){
      if (  m_grid_cells_active[i]==1){
        m_grid_cells_active[i]=0;
      }else{
        m_grid_cells_active[i]=1;
      }
    }
  }
  std::cout << "emited that the grid changed" << std::endl;
  emit grid_changed_signal();
}




void Model::wrap_grid(){
  std::cout << "wrapping the grid" << std::endl;

  //Generate 4 points from th cell
  //Find the nearest point in the unwrapped points with its index
  //That index will give the point in the wrapped ones

  create_point_cloud();
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (m_points_unwrapped_full_cloud);


  for (size_t cell_idx = 0; cell_idx < m_grid.size(); cell_idx++) {
    matrix_type points(4);
    points[0]=row_type {m_grid[cell_idx][0], m_grid[cell_idx][2] ,  m_grid[cell_idx][5]};
    points[1]=row_type {m_grid[cell_idx][1], m_grid[cell_idx][2] ,  m_grid[cell_idx][5]};
    points[2]=row_type {m_grid[cell_idx][1], m_grid[cell_idx][2] ,  m_grid[cell_idx][4]};
    points[3]=row_type {m_grid[cell_idx][0], m_grid[cell_idx][2] ,  m_grid[cell_idx][4]};

    row_type_i indexes(4);  //Indexes of the points that were the closest (for all 4 corners)

    matrix_type corners_world_unw(4);

    for (size_t p_idx = 0; p_idx < 4; p_idx++) {
      //get the closest point in the mesh that coresponds to the corner

      int K = 1;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);

      pcl::PointXYZ searchPoint;

      searchPoint.x=points[p_idx][0];
      searchPoint.y=points[p_idx][1];
      searchPoint.z=points[p_idx][2];

      // std::cout << "K nearest neighbor search at (" << searchPoint.x
      //         << " " << searchPoint.y
      //         << " " << searchPoint.z
      //         << ") with K=" << K << std::endl;

      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
          std::cout << "    "  <<   m_points_unwrapped_full_cloud->points[ pointIdxNKNSearch[i] ].x
                    << " " << m_points_unwrapped_full_cloud->points[ pointIdxNKNSearch[i] ].y
                    << " " << m_points_unwrapped_full_cloud->points[ pointIdxNKNSearch[i] ].z
                    << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
      }


      // double pos_display[3];
      // vtkInteractorObserver::ComputeWorldToDisplay (renderer, m_points_wrapped[pointIdxNKNSearch[0] ][0],
      //                                                         m_points_wrapped[pointIdxNKNSearch[0] ][1],
      //                                                         m_points_wrapped[pointIdxNKNSearch[0] ][2],
      //                                                         pos_display);
      corners_world_unw[p_idx]=row_type { m_points_wrapped[ pointIdxNKNSearch[0]][0],
                                          m_points_wrapped[ pointIdxNKNSearch[0]][1],
                                          m_points_wrapped[ pointIdxNKNSearch[0]][2], };


    }
    m_grid_wrapped.push_back(corners_world_unw);
  }


  std::cout << "finished wraping the grid" << std::endl;

}

void Model::create_point_cloud(){

  std::cout << "creating full point cloud" << std::endl;

  m_points_unwrapped_full_cloud->width    = m_points_unwrapped.size();
  m_points_unwrapped_full_cloud->height   = 1;
  m_points_unwrapped_full_cloud->is_dense = false;
  m_points_unwrapped_full_cloud->points.resize (m_points_unwrapped.size());

  for (size_t i = 0; i < m_points_unwrapped.size(); i++){
    m_points_unwrapped_full_cloud->points[i].x = m_points_unwrapped[i][0];
    m_points_unwrapped_full_cloud->points[i].y = m_points_unwrapped[i][1];
    m_points_unwrapped_full_cloud->points[i].z = m_points_unwrapped[i][2];
  }

  std::cout << "finished full point cloud" << std::endl;

}
