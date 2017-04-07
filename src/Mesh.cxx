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
  m_is_cylindrical(true),
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
  m_texture_ir(vtkSmartPointer<vtkTexture>::New()),

  m_high_cap_depth_color(255.0),
  m_low_cap_depth_color(0.0)
  {

}


void Mesh::set_mesh(vtkSmartPointer<vtkPolyData> polydata ){


  // vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  // points_active=vector_to_vtk(m_points_unwrapped);
  // m_wall->SetPoints(points_active);
  // m_wall->SetPolys(m_cells_wrapped);
  //
  //



  m_wall=auto_fix_pose(polydata);
  // m_wall=auto_fix_pose(smoothFilter->GetOutput());

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
  // m_radius           = estimate_radius(m_points_wrapped_ds);
  m_radius           = estimate_radius2();
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
  std::cout << "model::readinfo: nr cells: " <<  m_cells_wrapped->GetNumberOfCells()  << std::endl;


}

double Mesh::estimate_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr points ){

  std::cout << "estimating radius" << std::endl;
  double radius=0.0;

  //Get the points that are further away
  std::sort(m_points_wrapped_ds->points.begin(), m_points_wrapped_ds->points.end(), by_distance_center());

  int further_num;
  further_num=50;
  //A proportion is more robust
  // further_num=20*m_points_wrapped_ds->points.size()/100.0;
  further_num=100*m_points_wrapped_ds->points.size()/100.0;
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


double Mesh::estimate_radius2(){

  std::cout << "estimating radius" << std::endl;
  double radius=0.0;


  //get random points on the mesh
  int num_rand_points= 100*m_points_wrapped.size()/100.0;
  std::random_device rnd_device;
  std::mt19937 mersenne_engine(rnd_device());
  std::uniform_int_distribution<int> dist(0, m_points_wrapped.size()-1);
  auto gen = std::bind(dist, mersenne_engine);
  std::vector<int> points_ids(num_rand_points);
  std::generate(begin(points_ids), end(points_ids), gen);


  // //find the crossing of their normals. Save that position in x,y coordinates
  // for (size_t i = 0; i < points_ids.size(); i++) {
  //
  // }

  // All the objects needed
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  // Datasets
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (m_points_wrapped_ds);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);


  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (4.9, 5.0);
  seg.setInputCloud (m_points_wrapped_ds);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;


  // //Set also the center
  // m_bounds=m_wall->GetBounds();
  // m_center.resize(3);
  // m_center[0]=
  // m_center[1]=
  // m_center[2]=(m_bounds[5]+m_bounds[4])/2.0;

  radius= coefficients_cylinder->values[6];
  std::cout << "radius is " << radius << std::endl;

  return radius;












  // //Get the points that are further away
  // std::sort(m_points_wrapped_ds->points.begin(), m_points_wrapped_ds->points.end(), by_distance_center());
  //
  // int further_num;
  // further_num=50;
  // //A proportion is more robust
  // // further_num=20*m_points_wrapped_ds->points.size()/100.0;
  // further_num=100*m_points_wrapped_ds->points.size()/100.0;
  // std::vector< pcl::PointXYZ > further_points(m_points_wrapped_ds->points.begin(), m_points_wrapped_ds->points.begin() + further_num);
  //
  //
  // //Get the median of those distance as the radius
  // row_type distances(further_num);
  // for (size_t i = 0; i < further_points.size(); i++) {
  //   float dist= utils::distance(further_points[i], pcl::PointXYZ (0.0, 0.0, further_points[i].z));
  //   distances[i]=dist;
  // }
  //
  // radius= median(distances);
  //
  // std::cout << "radius is " << radius << std::endl;

  return radius;
}

double Mesh::estimate_circumference(double radius, double num_walls){
  double circumference;

  if (m_is_cylindrical){
    circumference    =2*M_PI*radius;
  }else{ //In the case of a non cylindrical one, the circumference is the perimeter of all the walls
    double angle_wall=360.0/(double)num_walls;   // see law of cosines
    angle_wall= angle_wall *M_PI/180.0;
    circumference =num_walls * sqrt (radius*radius +radius*radius -2*radius*radius* cos (angle_wall));
  }
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


  std::cout << "nr of color values is " << m_colors_active->GetNumberOfTuples()  << '\n';
  // if (m_colors_active->GetNumberOfTuples()!=0){
  //   m_wall->GetPointData()->SetScalars(m_colors_active);
  // }
  if (m_is_ply){
    m_wall->GetPointData()->SetScalars(m_colors_active);
  }
  std::cout << "Mesh get_vtk_mesh setting active colors with name" << m_colors_active->GetName()  << '\n';
  // m_wall->GetPointData()->SetActiveScalars("bright");


  if (this->m_is_unwrapped){
    delete_streched_trigs();
  }

  if (m_is_unwrapped){
    std::cout << "get_vtk_mesh, assigning  nr cells: " <<  m_cells_unwrapped->GetNumberOfCells()  << std::endl;
    m_wall->SetPolys(m_cells_unwrapped);
  }else{
    std::cout << "get_vtk_mesh, assigning  nr cells: " <<  m_cells_wrapped->GetNumberOfCells()  << std::endl;
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



vtkSmartPointer<vtkPolyData> Mesh::get_vtk_mesh_only_color(){
  if (m_is_ply){
    m_wall->GetPointData()->SetScalars(m_colors_active);
  }
  return m_wall;
}

void Mesh::delete_streched_trigs(){
  //get all the cells and calculate the perimeter. If the perimeter excedes a certain value, delete the cell (poly)
  if (m_deleted_streached_trigs)
    return;

  std::cout << "entered delete streched trigs" << '\n';
  m_deleted_streached_trigs=true;

  std::cout << "building links" << std::endl;
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

  m_wall->DeleteLinks();
}

//TODO Implement it
vtkSmartPointer<vtkPolyData> Mesh::auto_fix_pose( vtkSmartPointer<vtkPolyData> polydata){

  vtkSmartPointer<vtkPolyData> wall_aligned;
  wall_aligned=polydata;


  std::cout << "centering mesh" << std::endl;
  m_bounds=polydata->GetBounds();

  //If we already have a center of the mesh(given by estimate radius2) then we don't need to to this
  if (m_center.empty()){
    m_center.resize(3);
    m_center[0]=(m_bounds[1]+m_bounds[0])/2.0;
    m_center[1]=(m_bounds[3]+m_bounds[2])/2.0;
    m_center[2]=(m_bounds[5]+m_bounds[4])/2.0;
  }


  std::cout << "m_center is " << m_center[0] << " " << m_center[1] << " "  << m_center[2] << '\n';

  vtkSmartPointer<vtkTransform> translation =
    vtkSmartPointer<vtkTransform>::New();
  translation->Translate(-m_center[0], -m_center[1], -m_center[2]);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterCenter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  #if VTK_MAJOR_VERSION <= 5
    transformFilterCenter->SetInputConnection(polydata->GetProducerPort());
  #else
    transformFilterCenter->SetInputData(polydata);
  #endif
  transformFilterCenter->SetTransform(translation);
  transformFilterCenter->Update();
  wall_aligned=transformFilterCenter->GetOutput();









  //Fix orientation
  matrix_type normals_for_orientation;
  int downsample=10;

  double angle=0.0;

  vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
  vtk_normals->SetNumberOfComponents(3);
  vtk_normals->SetName("Normals");
  vtk_normals = vtkFloatArray::SafeDownCast(wall_aligned->GetPointData()->GetNormals());

  if (vtk_normals){
    std::cout << "autofix:: yes it has normals" << std::endl;
    std::cout << "autofix nr of normals" << vtk_normals->GetNumberOfTuples() << std::endl;
    normals_for_orientation=vtk_normal_tcoords_to_vector(vtk_normals);
  }else{
    std::cout << "autofix: it doesnt have normals" << std::endl;

    //We will estmate them

    vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
    #if VTK_MAJOR_VERSION <= 5
      normals_alg->SetInputConnection(wall_aligned->GetProducerPort());
    #else
      normals_alg->SetInputData(wall_aligned);
    #endif

    normals_alg->ComputePointNormalsOn();
    normals_alg->SplittingOff();
    normals_alg->ConsistencyOff();

    normals_alg->Update();

    vtkSmartPointer<vtkFloatArray> vtk_normals_estimated = vtkSmartPointer<vtkFloatArray>::New();
    vtk_normals_estimated->SetNumberOfComponents(3);
    vtk_normals_estimated->SetName("Normals");
    vtk_normals_estimated = vtkFloatArray::SafeDownCast(normals_alg->GetOutput()->GetPointData()->GetNormals());

    polydata->GetPointData()->SetNormals(vtk_normals_estimated);

    normals_for_orientation=vtk_normal_tcoords_to_vector(vtk_normals_estimated);
  }



  //Cluster the normals of the points.
  cv::Mat samples(normals_for_orientation.size()/downsample, 3, CV_32F);
  for( int y = 0; y < samples.rows; y++ ){
    for( int x = 0; x < samples.cols; x++ ){
      samples.at<float>(y,x)=normals_for_orientation[y*downsample][x];
    }
  }


  //TODO. Don't just hardcore the cluster count
  int cluster_count = 7;
  cv::Mat labels;
  int attempts = 10;
  cv::Mat centers;
  cv::kmeans(samples, cluster_count, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.001), attempts, cv::KMEANS_PP_CENTERS, centers );

  // //Get number of elements in each cluster
  // std::vector<int> labels_vec;
  // for (size_t i = 0; i < labels.rows; i++) {
  //   labels_vec.push_back(labels.at<int>(i,0));
  // }


  //Get the center,
  row_type selected_center(3);

  for (size_t i = 0; i < cluster_count; i++) {
    selected_center[0]+= fabs(centers.at<float>(i,0));
    selected_center[1]+= fabs(centers.at<float>(i,1));
    selected_center[2]+= fabs(centers.at<float>(i,2));
  }

  // std::cout << "----------------selected center is "<< selected_center[0] << " "  <<selected_center[1] << " " <<selected_center[2]<< std::endl;


  //If 2 is smallest -> its ok (the central axis is through the z axis)
  //if 1 is the smallest then rotate 90 in x axis
  //if 0 is the smallest rotate 90 degreen in y axis


  vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();

  std::vector<double>::iterator result = std::min_element(std::begin(selected_center), std::end(selected_center));
  int index=std::distance(std::begin(selected_center), result);
  // std::cout << "min element at: " << std::distance(std::begin(selected_center), result);

  if (index==0){
    std::cout << "ORIENT: Rotatin 90 in Y" << std::endl;
    trans->RotateY(90);
  }else if(index==1){
    std::cout << "ORIENT: Rotatin 90 in X" << std::endl;
    trans->RotateX(90);
  }else if(index==2){
    std::cout << "ORIENT: Orientation is correct" << std::endl;
    //I'ts in the correct orientation so don't do anything
    trans->RotateX(0);
  }



  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

  #if VTK_MAJOR_VERSION <= 5
      transformFilter->SetInputConnection(wall_aligned->GetProducerPort());
  #else
      transformFilter->SetInputData(wall_aligned);
  #endif


  transformFilter->SetTransform(trans);
  transformFilter->Update();

  wall_aligned=transformFilter->GetOutput();

  // return wall_aligned;


  //Rotate so as to get the 8th wall in the middle
  vtkSmartPointer<vtkTransform> trans2 = vtkSmartPointer<vtkTransform>::New();
  trans2->RotateZ(90);
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  #if VTK_MAJOR_VERSION <= 5
      transformFilter2->SetInputConnection(wall_aligned->GetProducerPort());
  #else
      transformFilter2->SetInputData(wall_aligned);
  #endif
  transformFilter2->SetTransform(trans2);
  transformFilter2->Update();

  std::cout << "finished autofixing the orientation" << std::endl;




  wall_aligned= transformFilter2->GetOutput();


  //Fix orientation so that the principal axis is towards z and also center the mesh

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

int Mesh::get_planes(std::vector<plane_struct>& planes, const pcl::PointCloud<pcl::PointXYZ>::Ptr points_wrapped_ds, const int num_walls){
  int cluster_count = num_walls;

  //vector of planes that will be ftted to the clouds
  planes.clear();
  planes.resize(num_walls);
  for (size_t i = 0; i < num_walls; i++) {
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

  int i=0;

  for (size_t i = 0; i < num_walls; i++) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (points_wrapped_ds);
    seg.segment (*inliers, (planes[i].coef));
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      return 1;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (points_wrapped_ds);
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
    Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ) ,
                        0.0,
                        0.0);

    double dot= planes[i].normal.dot(dir);

    if (dot<0.0){
      std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
      planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
    }
  }
}

int Mesh::get_planes(std::vector<plane_struct>& planes,
                     const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustered_clouds_not_flattened){


  int cluster_count=clustered_clouds_not_flattened.size();

  //----------------------------------------
   //Decimate the clouds
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
   for (size_t clust = 0; clust < cluster_count; clust++) {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
     clustered_clouds_ds.push_back(cloud);
   }
   for (size_t clust = 0; clust < cluster_count; clust++) {
     pcl::VoxelGrid<pcl::PointXYZ> ds;  //create downsampling filter
     ds.setInputCloud (clustered_clouds_not_flattened[clust]);
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

     seg2.setInputCloud (clustered_clouds_not_flattened[clust]);
     seg2.segment (*inliers2, (planes[clust].coef));

     if (inliers2->indices.size () == 0)
     {
       PCL_ERROR ("Could not estimate a planar model for the given dataset.");
     }



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
    Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ),
                        0.0,
                        0.0);

    double dot= planes[i].normal.dot(dir);

    if (dot<0.0){
      std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
      planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
    }
  }


  //-----------

}

void Mesh::sort_planes_by_angle(std::vector<plane_struct>& planes){
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
}

void Mesh::assing_points_to_planes(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustered_clouds_not_flattened,
                                   matrix_type_i& clustered_points_indices,
                                   const std::vector<plane_struct>& planes,
                                   const matrix_type& points_wrapped){

  clustered_points_indices.resize(planes.size());


  //Build a vector of cloud points
  clustered_clouds_not_flattened.clear();
  for (size_t clust = 0; clust < planes.size(); clust++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   clustered_clouds_not_flattened.push_back(cloud);
  }




  int clust_idx;
  //For each point in the mesh get the plane that is closest to it.
  for (size_t i = 0; i < points_wrapped.size(); i++) {

    int closest_plane_idx=-1;
    double distance=std::numeric_limits<double>::max();

    for (size_t plane_idx = 0; plane_idx < planes.size(); plane_idx++) {
      pcl::PointXYZ p;
      p.x=points_wrapped[i][0];
      p.y=points_wrapped[i][1];
      p.z=points_wrapped[i][2];
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
    pt.x=points_wrapped[i][0];
    pt.y=points_wrapped[i][1];
    pt.z=points_wrapped[i][2];


    clust_idx=planes[closest_plane_idx].index_cluster;
    clustered_clouds_not_flattened[clust_idx]->points.push_back(pt);
    clustered_points_indices[clust_idx].push_back(i);
  }

  std::cout << "finished clutering points" << std::endl;

  for (size_t i = 0; i < planes.size(); i++) {
    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
    // viewer.showCloud (clustered_clouds_not_flattened[i]);
    // while (!viewer.wasStopped ())
    // {
    // }
  }

}

void Mesh::flatten_clustered_points(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustered_clouds_flattened,
                                    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustered_clouds_not_flattened,
                                    const std::vector<plane_struct>& planes,
                                    const double circumference){


  //Create the flattened cloud
  for (size_t clust = 0; clust < planes.size(); clust++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   clustered_clouds_flattened.push_back(cloud);
  }


  // go through the planes and calculate the rotation translation matrix to map them
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
    end_point[0]= planes[i].angle * circumference - offset;
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

    pcl::transformPointCloud (*(clustered_clouds_not_flattened[clust_idx]), *(clustered_clouds_flattened[clust_idx]), transform_center);
    pcl::transformPointCloud (*(clustered_clouds_not_flattened[clust_idx]), *(clustered_clouds_flattened[clust_idx]), transform_2);

  }

}

void Mesh::tighten_clustered_points(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustered_clouds_flattened){


  int cluster_count =clustered_clouds_flattened.size();
  //
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
    double min_x_cur=std::numeric_limits<double>::max();
    double max_x_cur=std::numeric_limits<double>::min();
    for (size_t i_idx = 0; i_idx < clustered_clouds_flattened[clust]->points.size (); i_idx++) {
      if ( clustered_clouds_flattened[clust]->points[i_idx].x > max_x_cur  ){
        max_x_cur=clustered_clouds_flattened[clust]->points[i_idx].x;
      }
      if ( clustered_clouds_flattened[clust]->points[i_idx].x < min_x_cur  ){
        min_x_cur=clustered_clouds_flattened[clust]->points[i_idx].x;
      }

    }
    // std::cout << "cluster " << clust  << std::endl;
    // std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;

    min_max[clust]=row_type {min_x_cur, max_x_cur};  //there are now unordered
  }


  //Now that we have the min and max, move them
  std::cout << "movements" << std::endl;
  row_type movements(cluster_count, 0.0);
  for (size_t i = 0; i < cluster_count; i++) {
    int clust_idx= i ;
    if (i==0){
      movements[clust_idx]= 0.0 - min_max[clust_idx][0];
    }else{
      movements[clust_idx]=0.0;
      movements[clust_idx]+= movements[i-1];
      movements[clust_idx]+= (min_max[i-1][1] - min_max[clust_idx][0] );
    }
    std::cout << "clust index " << clust_idx << std::endl;
    std::cout << "movemnt: " << movements[clust_idx] << std::endl;
  }

  std::cout << "move them given the min and max" << std::endl;
  for (size_t i = 0; i < cluster_count; i++) {
    int clust_idx= i;



    std::cout << "plane: " << i << std::endl;
    std::cout << "min max: " << min_max[clust_idx][0] << " " << min_max[clust_idx][1] << std::endl;



    //Move it
    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // transform.translation() << min_x_cur - max_x_prev, 0.0, 0.0;
    // pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform);
    for (size_t p_idx = 0; p_idx < clustered_clouds_flattened[clust_idx]->points.size(); p_idx++) {
      clustered_clouds_flattened[clust_idx]->points[p_idx].x = clustered_clouds_flattened[clust_idx]->points[p_idx].x + movements[clust_idx];
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



}

void Mesh::write_clustered_points(matrix_type& points_unwrapped,
                                  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clustered_clouds_flattened,
                                  const matrix_type_i&  clustered_points_indices,
                                  const bool deform_walls){

  //get nr of points to write
  int nr_points=0;
  for (size_t i = 0; i < clustered_clouds_flattened.size(); i++) {
    nr_points+= clustered_clouds_flattened[i]->points.size();
  }
  int cluster_count=clustered_clouds_flattened.size();

  //write the unwrapped points
  points_unwrapped.resize(nr_points);
  for (size_t i = 0; i < points_unwrapped.size(); i++) {
    points_unwrapped[i].resize(3);
  }

  for (size_t clust = 0; clust < cluster_count; clust++) {
    int idx=0;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (clustered_clouds_flattened[clust]);

    for (size_t i = 0; i < clustered_clouds_flattened[clust]->points.size(); i++) {

      //we have a point and an index to insert it to, move the index until it winds the fisrt label
     //  while(labels.at<int>(idx,0)!=clust){
     //    idx++;
     //  }

     idx= clustered_points_indices[clust][i];

     points_unwrapped[idx][0]= clustered_clouds_flattened[clust]->points[i].x;
     points_unwrapped[idx][1]= clustered_clouds_flattened[clust]->points[i].y;
     points_unwrapped[idx][2]= clustered_clouds_flattened[clust]->points[i].z;


      //Need to check the nearest neightbours to fix the y (ditance to the plane)
      pcl::PointXYZ searchPoint;
      searchPoint=clustered_clouds_flattened[clust]->points[i];


      if (deform_walls){
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
            avg_dist+=clustered_clouds_flattened[clust]->points[ pointIdxNKNSearch[p] ].y;
          }
        }

        avg_dist=avg_dist/pointIdxNKNSearch.size ();
        points_unwrapped[idx][1]=points_unwrapped[idx][1]-avg_dist;
      }






      //clap the distance
      // if ( fabs(m_points_unwrapped[idx][1]) > 0.1 ){
      //   m_points_unwrapped[idx][1]=0.0;
      //   clustered_clouds[clust]->points[i].y=0.0;
      // }

    }
  }
}



void Mesh::GetConnectedVertices(vtkSmartPointer<vtkPolyData> mesh, int seed, row_type_i& connectedVertices){

  //get all cells that vertex 'seed' is a part of
  vtkSmartPointer<vtkIdList> cellIdList = vtkSmartPointer<vtkIdList>::New();
  mesh->GetPointCells(seed, cellIdList);

  //cout << "There are " << cellIdList->GetNumberOfIds() << " cells that use point " << seed << endl;

  //loop through all the cells that use the seed point
  for(vtkIdType i = 0; i < cellIdList->GetNumberOfIds(); i++)
    {

    vtkCell* cell = mesh->GetCell(cellIdList->GetId(i));
    //cout << "The cell has " << cell->GetNumberOfEdges() << " edges." << endl;

    //if the cell doesn't have any edges, it is a line
    if(cell->GetNumberOfEdges() <= 0)
      {
      continue;
      }

    for(vtkIdType e = 0; e < cell->GetNumberOfEdges(); e++)
      {
      vtkCell* edge = cell->GetEdge(e);

      vtkIdList* pointIdList = edge->GetPointIds();
      //cout << "This cell uses " << pointIdList->GetNumberOfIds() << " points" << endl;
      /*
      for(vtkIdType p = 0; p < pointIdList->GetNumberOfIds(); p++)
        {
        cout << "Edge " << i << " uses point " << pointIdList->GetId(p) << endl;
        }
        */
      if(pointIdList->GetId(0) == seed || pointIdList->GetId(1) == seed)
        {
        if(pointIdList->GetId(0) == seed)
          {
          connectedVertices.push_back(pointIdList->GetId(1));
          }
        else
          {
          connectedVertices.push_back(pointIdList->GetId(0));
          }
        }
      }


    }

    //cout << "There are " << connectedVertices->GetNumberOfIds() << " points connected to point " << seed << endl;
}


row_type_i Mesh::GetConnectedVertices2(vtkSmartPointer<vtkPolyData> mesh, int id) {
  row_type_i connectedVertices;

  //get all cells that vertex 'id' is a part of
  vtkSmartPointer<vtkIdList> cellIdList =
      vtkSmartPointer<vtkIdList>::New();
  mesh->GetPointCells(id, cellIdList);

  /*
  cout << "Vertex 0 is used in cells ";
  for(vtkIdType i = 0; i < cellIdList->GetNumberOfIds(); i++)
    {
    cout << cellIdList->GetId(i) << ", ";
    }
  cout << endl;
  */

  for(vtkIdType i = 0; i < cellIdList->GetNumberOfIds(); i++)
    {
    //cout << "id " << i << " : " << cellIdList->GetId(i) << endl;

    vtkSmartPointer<vtkIdList> pointIdList =
      vtkSmartPointer<vtkIdList>::New();
    mesh->GetCellPoints(cellIdList->GetId(i), pointIdList);

    //cout << "End points are " << pointIdList->GetId(0) << " and " << pointIdList->GetId(1) << endl;

    if(pointIdList->GetId(0) != id)
      {
      //cout << "Connected to " << pointIdList->GetId(0) << endl;
      connectedVertices.push_back(pointIdList->GetId(0));
      }
    else
      {
      //cout << "Connected to " << pointIdList->GetId(1) << endl;
      connectedVertices.push_back(pointIdList->GetId(1));
      }
    }

  return connectedVertices;
}

double Mesh::compute_average_height(vtkSmartPointer<vtkPolyData> mesh, int search_point_idx, double radius){
  double sigma= radius/3;

  row_type_i neighbours_ids;
  std::queue<int> expansion_front;
  expansion_front.push(search_point_idx);
  neighbours_ids.push_back(search_point_idx);
  // std::cout << "----------connected from point " << search_point_idx << '\n';
  // utils::tick();
  while (!expansion_front.empty()){

    row_type_i connected_vertices;
    int expand_idx=expansion_front.front();
    expansion_front.pop();
    // GetConnectedVertices(mesh,expand_idx,connected_vertices);
    connected_vertices = GetConnectedVertices2(mesh, expand_idx);

    // std::cout << "initial connected is "<< connected_vertices.size() << '\n';

    //if the connected vertices are not in neighbours and also are within the radius, add to expansion front
    for (size_t c_idx = 0; c_idx < connected_vertices.size(); c_idx++) {
      bool already_added=false;
      //check if its already in the neighbours
      for (size_t n_idx = 0; n_idx < neighbours_ids.size(); n_idx++) {
        if (neighbours_ids[n_idx]==connected_vertices[c_idx]){
          already_added=true;
        }
      }

      if (!already_added){
        double dist=utils::dist (m_points_unwrapped[search_point_idx], m_points_unwrapped[ connected_vertices[c_idx]]);
        if (dist < radius){
          expansion_front.push(connected_vertices[c_idx]);
          neighbours_ids.push_back(connected_vertices[c_idx]);
        }
      }

    }

    // std::cout << "connected_vertices size is " << connected_vertices.size() << '\n';
  }

  // utils::tock <std::chrono::milliseconds> ("Get all neighbours in radius");
  std::cout << "total neighbours insize radius is " << neighbours_ids.size() << '\n';
  double weight_cum=0.0;
  double avg_dist=0.0;
  for (size_t p = 0; p < neighbours_ids.size (); ++p){
    // //average the dist of the K nearest neghbours

    // double dist=utils::distance(searchPoint, cloud->points[ pointIdxNKNSearch[p] ]);
    //
    // //The weight givn to each point in the neightbourhood can be a gaussian or simply a uniform weight
    // // double weight=1.0
    // double weight= (1/std::sqrt(2*M_PI*sigma*sigma) ) * std::exp( - (dist*dist)/(2*sigma*sigma) );
    //
    // weight_cum+=weight;
    // avg_dist+=weight*cloud->points[ pointIdxNKNSearch[p] ].y;

     double dist=utils::dist(m_points_unwrapped[search_point_idx], m_points_unwrapped[ neighbours_ids[p] ]);
     double weight= (1/std::sqrt(2*M_PI*sigma*sigma) ) * std::exp( - (dist*dist)/(2*sigma*sigma) );
     weight_cum+=weight;
     avg_dist+=weight*m_points_unwrapped[ neighbours_ids[p] ][1];
  }

  avg_dist=avg_dist/weight_cum;

  return avg_dist;


}


double Mesh::compute_average_height_kdtree(pcl::KdTreeFLANN<pcl::PointXYZ> kdtree, int point_idx, double radius){

  double sigma= radius/3;

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  double avg_dist=0.0;
  double weight_cum=0.0;

  pcl::PointXYZ searchPoint;
  searchPoint.x=m_points_unwrapped[point_idx][0];
  searchPoint.y=m_points_unwrapped[point_idx][1];
  searchPoint.z=m_points_unwrapped[point_idx][2];
  // searchPoint=cloud->points[point_idx];

  if (  kdtree.radiusSearch (searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
   for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p){
      // //average the dist of the K nearest neghbours

      pcl::PointXYZ neighbour_point;
      neighbour_point.x=m_points_unwrapped[ pointIdxNKNSearch[p]][0];
      neighbour_point.y=m_points_unwrapped[ pointIdxNKNSearch[p]][1];
      neighbour_point.z=m_points_unwrapped[ pointIdxNKNSearch[p]][2];

      double dist=utils::distance(searchPoint, neighbour_point);

      //The weight givn to each point in the neightbourhood can be a gaussian or simply a uniform weight
      // double weight=1.0
      double weight= (1/std::sqrt(2*M_PI*sigma*sigma) ) * std::exp( - (dist*dist)/(2*sigma*sigma) );

      weight_cum+=weight;
      avg_dist+=weight*neighbour_point.y;
    }
  }

  avg_dist=avg_dist/weight_cum;
  return avg_dist;


}



void Mesh::precompute_all_neighbours(vtkSmartPointer<vtkPolyData> mesh, matrix_type_i& neighbours_ids_all){

  neighbours_ids_all.clear();

  for (size_t i = 0; i < m_points_unwrapped.size(); i++) {
    row_type_i connected_vertices;
    GetConnectedVertices(mesh,i,connected_vertices);
    neighbours_ids_all.push_back(connected_vertices);
  }
}


void Mesh::unwrap_cyl(){
  //------CYLINDRICAL-------------------
  m_points_unwrapped.clear();

  m_circumference    = estimate_circumference (m_radius, m_num_walls);
  m_angles              =computeAngles(m_points_wrapped);
  m_distances_to_radius = computeDistancesToRadius(m_points_wrapped, m_radius);


  // Write them to a cloud so that we can flatten them better by flattening the low frequency detail and retain the high frequency
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < m_points_wrapped.size(); i++) {

    pcl::PointXYZ pt;
    pt.x=m_angles[i] *m_circumference;
    pt.y=m_distances_to_radius[i];
    pt.z=m_points_wrapped[i][2];

    cloud->points.push_back(pt);
  }

  //write the cloud back to the m_points_unwrapped but do it while removing the low frequency mesh geometry
  // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  // kdtree.setInputCloud (cloud);
  m_points_unwrapped.resize(m_points_wrapped.size());
  for (size_t i = 0; i < m_points_wrapped.size(); i++) {

      // pcl::PointXYZ searchPoint;
      // searchPoint=cloud->points[i];

      m_points_unwrapped[i].resize(3);
      m_points_unwrapped[i][0]=m_angles[i] *m_circumference;
      m_points_unwrapped[i][1]=m_distances_to_radius[i];
      m_points_unwrapped[i][2]=m_points_wrapped[i][2];
  }

      // if (true){
      //   int K = 50;
      //   // double radius=0.1;
      //   // double radius=0.05;
      //   double radius=0.45;
      //   double sigma= radius/3;
      //
      //   std::vector<int> pointIdxNKNSearch(K);
      //   std::vector<float> pointNKNSquaredDistance(K);
      //
      //   double avg_dist=0.0;
      //   double weight_cum=0.0;
      //
      //   if (  kdtree.radiusSearch (searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      //   {
      //    for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p){
      //       // //average the dist of the K nearest neghbours
      //
      //       double dist=utils::distance(searchPoint, cloud->points[ pointIdxNKNSearch[p] ]);
      //
      //       //The weight givn to each point in the neightbourhood can be a gaussian or simply a uniform weight
      //       // double weight=1.0
      //       double weight= (1/std::sqrt(2*M_PI*sigma*sigma) ) * std::exp( - (dist*dist)/(2*sigma*sigma) );
      //
      //       weight_cum+=weight;
      //       avg_dist+=weight*cloud->points[ pointIdxNKNSearch[p] ].y;
      //     }
      //   }


  //second way of doing it by looking at the conectivity
  // if (true){
  //   //write points to and cells to mesh
  //   vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  //   points_active=vector_to_vtk(m_points_unwrapped);
  //   m_wall->SetPoints(points_active);
  //   m_wall->SetPolys(m_cells_wrapped); //Write the wrapped cells because we don't have yet the unwrapped ones (we didnt delete strechted trig)
  //
  //   // vtkSmartPointer<vtkExtractEdges> extractEdges = vtkSmartPointer<vtkExtractEdges>::New();
  //   // extractEdges->SetInputConnection(m_wall->GetProducerPort());
  //   // extractEdges->Update();
  //   // std::vector<vtkSmartPointer<vtkPolyData>> meshes(4);
  //   // meshes[0]=extractEdges->GetOutput();
  //   // for (size_t i = 0; i < 4; i++) {
  //   //   meshes[i]=vtkSmartPointer<vtkPolyData>::New();
  //   //   meshes[i]->DeepCopy(meshes[0]);
  //   // }
  //   // std::cout << "starting " << '\n';
  //   // vtkSmartPointer<vtkPolyData> mesh = extractEdges->GetOutput();
  //   vtkSmartPointer<vtkPolyData> mesh = m_wall;
  //
  //   // double radius=0.45;
  //   double radius=0.25;
  //   double radius_squared=radius*radius;
  //   double sigma= radius/3;
  //
  //   matrix_type_i neighbours_ids_all;
  //   precompute_all_neighbours(mesh, neighbours_ids_all);
  //   std::cout << "done precomputing" << '\n';
  //
  //   #pragma omp parallel for
  //   for (size_t i = 0; i < m_points_wrapped.size(); i++) {
  //
  //
  //     double avg_dist=0.0;
  //     double weight_cum=0.0;
  //
  //     int thread_num = omp_get_thread_num();
  //     // std::cout << "hello from thread" << thread_num << '\n';
  //
  //
  //
  //     //get all the neighbours of searchPoint
  //     row_type_i neighbours_ids;
  //     std::queue<int> expansion_front;
  //     expansion_front.push(i);
  //     neighbours_ids.push_back(i);
  //     // std::cout << "----------connected from point " << i << '\n';
  //     // utils::tick();
  //     while (!expansion_front.empty()){
  //
  //       row_type_i connected_vertices;
  //       int expand_idx=expansion_front.front();
  //       expansion_front.pop();
  //       // GetConnectedVertices(mesh,expand_idx,connected_vertices);
  //       connected_vertices=neighbours_ids_all[expand_idx];
  //       // GetConnectedVertices(meshes[thread_num],expand_idx,connected_vertices);
  //       // std::cout << measure<>::execution( GetConnectedVertices2(mesh, expand_idx) ) << std::endl;
  //
  //
  //       // connected_vertices = GetConnectedVertices2(meshes[thread_num], expand_idx);
  //
  //
  //       //if the connected vertices are not in neighbours and also are within the radius, add to expansion front
  //       for (size_t c_idx = 0; c_idx < connected_vertices.size(); c_idx++) {
  //         bool already_added=false;
  //         //check if its already in the neighbours
  //         for (size_t n_idx = 0; n_idx < neighbours_ids.size(); n_idx++) {
  //           if (neighbours_ids[n_idx]==connected_vertices[c_idx]){
  //             already_added=true;
  //           }
  //         }
  //
  //         if (!already_added){
  //           double dist=utils::dist (m_points_unwrapped[i], m_points_unwrapped[ connected_vertices[c_idx]]);
  //           if (dist < radius){
  //             expansion_front.push(connected_vertices[c_idx]);
  //             neighbours_ids.push_back(connected_vertices[c_idx]);
  //           }
  //         }
  //
  //       }
  //
  //       // std::cout << "connected_vertices size is " << connected_vertices.size() << '\n';
  //     }
  //
  //     // utils::tock <std::chrono::milliseconds> ("Get all neighbours in radius");
  //     // std::cout << "total neighbours insize radius is " << neighbours_ids.size() << '\n';
  //     for (size_t p = 0; p < neighbours_ids.size (); ++p){
  //       // //average the dist of the K nearest neghbours
  //
  //       // double dist=utils::distance(searchPoint, cloud->points[ pointIdxNKNSearch[p] ]);
  //       //
  //       // //The weight givn to each point in the neightbourhood can be a gaussian or simply a uniform weight
  //       // // double weight=1.0
  //       // double weight= (1/std::sqrt(2*M_PI*sigma*sigma) ) * std::exp( - (dist*dist)/(2*sigma*sigma) );
  //       //
  //       // weight_cum+=weight;
  //       // avg_dist+=weight*cloud->points[ pointIdxNKNSearch[p] ].y;
  //
  //        double dist=utils::dist(m_points_unwrapped[i], m_points_unwrapped[ neighbours_ids[p] ]);
  //        double weight= (1/std::sqrt(2*M_PI*sigma*sigma) ) * std::exp( - (dist*dist)/(2*sigma*sigma) );
  //        weight_cum+=weight;
  //        avg_dist+=weight*m_points_unwrapped[ neighbours_ids[p] ][1];
  //     }
  //
  //     avg_dist=avg_dist/weight_cum;
  //     // m_points_unwrapped[i][1]=m_points_unwrapped[i][1]-avg_dist;
  //     m_points_unwrapped[i][1]-=avg_dist;
  //
  //   }
  // }



  //third way of doing it by doing it sparsely on only a few points
  // if (true){
  //   // int num_support_points=0.1*m_points_unwrapped.size()/100.0;
  //   int num_support_points=300;
  //   std::cout << "num of support points" << num_support_points << '\n';
  //   double radius=0.45;
  //
  //   //get random indeces for points
  //   std::random_device rnd_device;
  //   std::mt19937 mersenne_engine(rnd_device());
  //   std::uniform_int_distribution<int> dist(0, m_points_unwrapped.size()-1);
  //   auto gen = std::bind(dist, mersenne_engine);
  //   std::vector<int> support_points_ids(num_support_points);
  //   std::generate(begin(support_points_ids), end(support_points_ids), gen);
  //
  //
  //   //get the mesh
  //   vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  //   points_active=vector_to_vtk(m_points_unwrapped);
  //   m_wall->SetPoints(points_active);
  //   m_wall->SetPolys(m_cells_wrapped); //Write the wrapped cells because we don't have yet the unwrapped ones (we didnt delete strechted trig)
  //
  //   // vtkSmartPointer<vtkExtractEdges> extractEdges = vtkSmartPointer<vtkExtractEdges>::New();
  //   // extractEdges->SetInputConnection(m_wall->GetProducerPort());
  //   // extractEdges->Update();
  //   // vtkSmartPointer<vtkPolyData> mesh = extractEdges->GetOutput();
  //    vtkSmartPointer<vtkPolyData> mesh=m_wall;
  //
  //   //get the average height of them
  //   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  //   kdtree.setInputCloud (cloud);
  //   row_type support_points_avg_heights(support_points_ids.size(),0);
  //   for (size_t i = 0; i < support_points_ids.size(); i++) {
  //     support_points_avg_heights[i]=compute_average_height(mesh,support_points_ids[i],radius);
  //     // support_points_avg_heights[i]=compute_average_height_kdtree(cloud,support_points_ids[i],radius);
  //     // support_points_avg_heights[i]=compute_average_height_kdtree(kdtree,support_points_ids[i],radius);
  //   }
  //
  //   //create point cloud with point having x,y as the original points and y being the average height
  //   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_support_points(new pcl::PointCloud<pcl::PointXYZ>);
  //   // for (size_t i = 0; i < support_points_ids.size(); i++) {
  //   //   pcl::PointXYZ pt;
  //   //   pt.x=m_points_unwrapped[support_points_ids[i]][0];
  //   //   pt.y=support_points_avg_heights[i];
  //   //   pt.z=m_points_unwrapped[support_points_ids[i]][2];
  //   //   cloud_support_points->points.push_back(pt);
  //   // }
  //
  //   // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  //   //   viewer.showCloud (cloud_support_points);
  //   //   while (!viewer.wasStopped ())
  //   //   {
  //   //   }
  //
  //   //loop through all the unwrapped points
  //     //for each one of them interpolate between all points of that cloud and weight the y values of them
  //     //that is going to be the average height of this points and we substract it from it
  //   // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  //   // kdtree.setInputCloud (cloud_support_points);
  //   // for (size_t i = 0; i < m_points_unwrapped.size(); i++) {
  //   //   pcl::PointXYZ searchPoint;
  //   //   searchPoint.x=m_points_unwrapped[i][0];
  //   //   searchPoint.y=m_points_unwrapped[i][1];
  //   //   searchPoint.z=m_points_unwrapped[i][2];
  //   //   int K = 50;
  //   //   // double radius=0.1;
  //   //   // double radius=0.05;
  //   //   double sigma= radius/3;
  //   //
  //   //   std::vector<int> pointIdxNKNSearch(K);
  //   //   std::vector<float> pointNKNSquaredDistance(K);
  //   //
  //   //   double avg_dist=0.0;
  //   //   double weight_cum=0.0;
  //   //
  //   //   if (  kdtree.radiusSearch (searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
  //   //     for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p){
  //   //        // //average the dist of the K nearest neghbours
  //   //
  //   //        double dist=utils::distance(searchPoint, cloud_support_points->points[ pointIdxNKNSearch[p] ]);
  //   //
  //   //        //The weight givn to each point in the neightbourhood can be a gaussian or simply a uniform weight
  //   //        // double weight=1.0
  //   //        double weight= (1/std::sqrt(2*M_PI*sigma*sigma) ) * std::exp( - (dist*dist)/(2*sigma*sigma) );
  //   //
  //   //        weight_cum+=weight;
  //   //        avg_dist+=weight*cloud_support_points->points[ pointIdxNKNSearch[p] ].y;
  //   //     }
  //   //   }
  //   //   avg_dist=avg_dist/weight_cum;
  //   //   std::cout << "avg dist is " << avg_dist << '\n';
  //   //   m_points_unwrapped[i][1]-=avg_dist;
  //   // }
  //   std::cout << "start intepolating" << '\n';
  //   #pragma omp parallel for
  //   for (size_t i = 0; i < m_points_unwrapped.size(); i++) {
  //     double avg_dist=0.0;
  //     double weight_cum=0.0;
  //     for (size_t n = 0; n < support_points_ids.size(); n++) {
  //       double dist=utils::dist_no_y(m_points_unwrapped[i], m_points_unwrapped[support_points_ids[n]]);
  //       dist=dist*dist*dist*dist*dist*dist*dist*dist*dist;
  //       dist+=0.01; //To deal with the case when the distance is 0
  //       double weight= 1.0/dist;
  //       weight_cum+=weight;
  //       avg_dist+=weight*support_points_avg_heights[n];
  //     }
  //     avg_dist=avg_dist/weight_cum;
  //     // if (weight_cum==0.0){
  //     //   avg_dist=0.0;
  //     // }
  //     m_points_unwrapped[i][1]-=avg_dist;
  //   }
  //
  //
  // }


  //FORTH and final way BFS but this time with more efficient checking if it was already added
  // if (true){
  //   //write points to and cells to mesh
  //   vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  //   points_active=vector_to_vtk(m_points_unwrapped);
  //   m_wall->SetPoints(points_active);
  //   m_wall->SetPolys(m_cells_wrapped); //Write the wrapped cells because we don't have yet the unwrapped ones (we didnt delete strechted trig)
  //   vtkSmartPointer<vtkPolyData> mesh = m_wall;
  //
  //   // double radius=0.45;
  //   double radius=0.15;
  //   double radius_squared=radius*radius;
  //   double sigma= radius/3;
  //
  //   matrix_type_i neighbours_ids_all;
  //   precompute_all_neighbours(mesh, neighbours_ids_all);
  //   std::cout << "done precomputing" << '\n';
  //
  //   std::vector<std::vector<bool>> already_checked(4); //Every thread has a an array of already checked points
  //   for (size_t i = 0; i < already_checked.size(); i++) {
  //     already_checked[i].resize(m_points_unwrapped.size());
  //     std::fill(already_checked[i].begin(), already_checked[i].end(), false);
  //   }
  //
  //   std::chrono::steady_clock::time_point begin;
  //
  //   #pragma omp parallel for num_threads(1)
  //   for (size_t i = 0; i < m_points_wrapped.size(); i++) {
  //
  //     if (i%1000==0){
  //       // std::cout << "reached point" << i << '\n';
  //       begin = std::chrono::steady_clock::now();
  //     }
  //
  //     double avg_dist=0.0;
  //     double weight_cum=0.0;
  //
  //     int thread_num = omp_get_thread_num();
  //     unsigned int size= m_points_unwrapped.size();
  //     already_checked[thread_num].assign(size, false);
  //     // std::fill(already_checked[thread_num].begin(), already_checked[thread_num].end(), false);
  //     // std::cout << "hello from thread" << thread_num << '\n';
  //
  //     //get all the neighbours of searchPoint
  //     row_type_i neighbours_ids;
  //     std::queue<int> expansion_front;
  //     expansion_front.push(i);
  //     neighbours_ids.push_back(i);
  //     // std::cout << "----------connected from point " << i << '\n';
  //     // utils::tick();
  //     while (!expansion_front.empty()){
  //
  //       row_type_i connected_vertices;
  //       int expand_idx=expansion_front.front();
  //       expansion_front.pop();
  //       // GetConnectedVertices(mesh,expand_idx,connected_vertices);
  //       connected_vertices=neighbours_ids_all[expand_idx];
  //       // GetConnectedVertices(meshes[thread_num],expand_idx,connected_vertices);
  //       // std::cout << measure<>::execution( GetConnectedVertices2(mesh, expand_idx) ) << std::endl;
  //
  //
  //       // connected_vertices = GetConnectedVertices2(meshes[thread_num], expand_idx);
  //
  //
  //       //if the connected vertices are not in neighbours and also are within the radius, add to expansion front
  //       for (size_t c_idx = 0; c_idx < connected_vertices.size(); c_idx++) {
  //         bool already_added=false;
  //         //check if its already in the neighbours
  //         // for (size_t n_idx = 0; n_idx < neighbours_ids.size(); n_idx++) {
  //         //   if (neighbours_ids[n_idx]==connected_vertices[c_idx]){
  //         //     already_added=true;
  //         //   }
  //         // }
  //         // if (  already_checked[thread_num][connected_vertices[c_idx]]){
  //         //   already_added=true;
  //         // }
  //
  //         already_added= already_checked[thread_num][connected_vertices[c_idx]];
  //
  //
  //         if (!already_added){
  //           double dist=utils::dist (m_points_unwrapped[i], m_points_unwrapped[ connected_vertices[c_idx]]);
  //           if (dist < radius){
  //             expansion_front.push(connected_vertices[c_idx]);
  //             neighbours_ids.push_back(connected_vertices[c_idx]);
  //             already_checked[thread_num][connected_vertices[c_idx]]=true;
  //           }
  //         }
  //
  //       }
  //
  //       // std::cout << "connected_vertices size is " << connected_vertices.size() << '\n';
  //     }
  //
  //     if (i%1000){
  //       // utils::tock <std::chrono::milliseconds> ("Get all neighbours in radius for 1000 points");
  //       std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
  //       std::cout << "TIME: " " = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;
  //       std::cout << "neight is " << neighbours_ids.size() << '\n';
  //     }
  //
  //     // utils::tock <std::chrono::milliseconds> ("Get all neighbours in radius");
  //     // std::cout << "total neighbours insize radius is " << neighbours_ids.size() << '\n';
  //
  //     // utils::tick();
  //     for (size_t p = 0; p < neighbours_ids.size (); ++p){
  //
  //        double dist=utils::dist(m_points_unwrapped[i], m_points_unwrapped[ neighbours_ids[p] ]);
  //        double weight= (1/std::sqrt(2*M_PI*sigma*sigma) ) * std::exp( - (dist*dist)/(2*sigma*sigma) );
  //        weight_cum+=weight;
  //        avg_dist+=weight*m_points_unwrapped[ neighbours_ids[p] ][1];
  //     }
  //     // utils::tock <std::chrono::milliseconds> ("Getting the average height");
  //
  //
  //     avg_dist=avg_dist/weight_cum;
  //     // m_points_unwrapped[i][1]=m_points_unwrapped[i][1]-avg_dist;
  //     m_points_unwrapped[i][1]-=avg_dist;
  //
  //   }
  // }



  //FIFTH way smooth it and then get the difference between points
  // vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  // points_active=vector_to_vtk(m_points_unwrapped);
  // m_wall->SetPoints(points_active);
  // m_wall->SetPolys(m_cells_wrapped);
  //
  //
  // vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
  // smoothFilter->SetInputConnection(m_wall->GetProducerPort());
  // // smoothFilter->SetNumberOfIterations(300);
  // // smoothFilter->SetRelaxationFactor(1.5);
  // smoothFilter->SetNumberOfIterations(300);
  // smoothFilter->SetRelaxationFactor(0.7);
  // smoothFilter->FeatureEdgeSmoothingOn();
  // smoothFilter->BoundarySmoothingOn();
  // smoothFilter->Update();
  //
  //
  // matrix_type m_points_unwrapped_smoothed= vtk_to_vector(smoothFilter->GetOutput()->GetPoints());
  // for (size_t i = 0; i < m_points_wrapped.size(); i++) {
  //   m_points_unwrapped[i][1]-=m_points_unwrapped_smoothed[i][1];
  // }




  // //assign the new points
  // m_points_unwrapped.resize(m_points_wrapped.size());
  // for (size_t i = 0; i < m_points_wrapped.size(); i++) {
  //   m_points_unwrapped[i].resize(3);
  //   m_points_unwrapped[i][0]=m_angles[i] *m_circumference;
  //   m_points_unwrapped[i][1]=m_distances_to_radius[i];
  //   m_points_unwrapped[i][2]=m_points_wrapped[i][2];
  // }
  std::cout << "finish asign points" << std::endl;

  return;

  //------END CYLINDRICAL --------------
}

void Mesh::unwrap_planar(){
  m_planes.clear();
  m_points_unwrapped.clear();
  m_deleted_streached_trigs=false;
  m_cells_unwrapped=vtkSmartPointer<vtkCellArray>::New();
  m_clustered_clouds_not_flattened.clear();


  m_circumference    = estimate_circumference (m_radius, m_num_walls);

  get_planes(m_planes, m_points_wrapped_ds, m_num_walls);//execute ransac iteratively for the num_walls we have to get a rough estimate of the planes
  sort_planes_by_angle(m_planes);     //Sort the planes by the angle between their normal and x axis.


  //HACK
  for (size_t i = 0; i < m_planes.size(); i++) {
    m_planes[i].index_cluster=i;
    m_planes[i].coef.values[3]=m_planes[i].coef.values[3]-0.1;  //HACK to move the planes closer to the center so that there are no points misclasified in the wrong plane
  }


  matrix_type_i clustered_points_indices; //indices to know from where do the points in the clustered clouds originate from in the original m_points_wrapped. We need to know it so we write in the same poisition in the m_points_unwrapped otherwise the mesh cell will not link the correct vertices.
  assing_points_to_planes(m_clustered_clouds_not_flattened, clustered_points_indices,  m_planes, m_points_wrapped);  //assign each of the wrapped points to one of the planes depending on the distance
  //save the assigned and clustred clouds before we do any transformation to them
  //robustify planes by fitting them again to the new clustered clouds
  get_planes(m_planes, m_clustered_clouds_not_flattened);
  sort_planes_by_angle(m_planes);


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_flattened;
  flatten_clustered_points(clustered_clouds_flattened, m_clustered_clouds_not_flattened, m_planes, m_circumference);
  tighten_clustered_points(clustered_clouds_flattened);
  write_clustered_points(m_points_unwrapped, clustered_clouds_flattened, clustered_points_indices, m_deform_walls);

  m_num_walls_baked=m_num_walls;
}

void Mesh::compute_unwrap(){


  if (m_is_cylindrical){
    unwrap_cyl();
  } else{
    unwrap_planar();
  }

  return;






  //need to get the circumference again because the number of walls may have changed and in the case of non conical, the circumference is the perimeter of the walls

  m_planes.clear();
  m_points_unwrapped.clear();
  m_deleted_streached_trigs=false;
  m_cells_unwrapped=vtkSmartPointer<vtkCellArray>::New();
  m_clustered_clouds_not_flattened.clear();


  m_circumference    = estimate_circumference (m_radius, m_num_walls);

  get_planes(m_planes, m_points_wrapped_ds, m_num_walls);//execute ransac iteratively for the num_walls we have to get a rough estimate of the planes
  sort_planes_by_angle(m_planes);     //Sort the planes by the angle between their normal and x axis.


  //HACK
  for (size_t i = 0; i < m_planes.size(); i++) {
    m_planes[i].index_cluster=i;
    m_planes[i].coef.values[3]=m_planes[i].coef.values[3]-0.1;  //HACK to move the planes closer to the center so that there are no points misclasified in the wrong plane
  }


  matrix_type_i clustered_points_indices; //indices to know from where do the points in the clustered clouds originate from in the original m_points_wrapped. We need to know it so we write in the same poisition in the m_points_unwrapped otherwise the mesh cell will not link the correct vertices.
  assing_points_to_planes(m_clustered_clouds_not_flattened, clustered_points_indices,  m_planes, m_points_wrapped);  //assign each of the wrapped points to one of the planes depending on the distance
  //save the assigned and clustred clouds before we do any transformation to them
  //robustify planes by fitting them again to the new clustered clouds
  get_planes(m_planes, m_clustered_clouds_not_flattened);
  sort_planes_by_angle(m_planes);


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_flattened;
  flatten_clustered_points(clustered_clouds_flattened, m_clustered_clouds_not_flattened, m_planes, m_circumference);
  tighten_clustered_points(clustered_clouds_flattened);
  write_clustered_points(m_points_unwrapped, clustered_clouds_flattened, clustered_points_indices, m_deform_walls);

  m_num_walls_baked=m_num_walls;

  //TODO calculate new normals for the unwrapped mesh



 //  //do some unwrap
 //  //Run ransca to get intial planes
 //  //each point will get clustered into the plane that is closest to it.
 //  int cluster_count = m_num_walls;
 //
 //  //vector of planes that will be ftted to the clouds
 //  planes.clear();
 //  planes.resize(m_num_walls);
 //  for (size_t i = 0; i < m_num_walls; i++) {
 //     planes[i].coef.values.resize(4);
 //     planes[i].index_cluster=i;
 //   }
 //
 //
 //  //Get the rough planes
 //  pcl::SACSegmentation<pcl::PointXYZ> seg;
 //  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
 //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
 //  seg.setOptimizeCoefficients (false);
 //  seg.setModelType (pcl::SACMODEL_PLANE);
 //  seg.setMethodType (pcl::SAC_RANSAC);
 //  seg.setMaxIterations (400);
 //  seg.setDistanceThreshold (0.03);
 //
 //  int i=0, nr_points = (int) m_points_wrapped_ds->points.size ();
 //
 //  while (i<m_num_walls)
 //  {
 //    // Segment the largest planar component from the remaining cloud
 //    seg.setInputCloud (m_points_wrapped_ds);
 //    seg.segment (*inliers, (planes[i].coef));
 //    if (inliers->indices.size () == 0)
 //    {
 //      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
 //      break;
 //    }
 //
 //    // Extract the planar inliers from the input cloud
 //    pcl::ExtractIndices<pcl::PointXYZ> extract;
 //    extract.setInputCloud (m_points_wrapped_ds);
 //    extract.setIndices (inliers);
 //    extract.setNegative (false);
 //
 //    // Get the points associated with the planar surface
 //    extract.filter (*cloud_plane);
 //    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
 //
 //    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
 //    // viewer.showCloud (cloud_plane);
 //    // while (!viewer.wasStopped ())
 //    // {
 //    // }
 //
 //    // Remove the planar inliers, extract the rest
 //    extract.setNegative (true);
 //    extract.filter (*cloud_f);
 //    *m_points_wrapped_ds = *cloud_f;
 //
 //    i++;
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
 //  for (size_t i = 0; i < planes.size(); i++) {
 //    planes[i].index_cluster=i;
 //    planes[i].coef.values[3]=planes[i].coef.values[3]-0.1;  //HACK
 //  }
 //
 //
 // //-------------------
 //
 //  matrix_type_i clustered_points_indices(m_num_walls);
 //
 //
 //  //Build a vector of cloud points
 //  clustered_clouds.clear();
 //  for (size_t clust = 0; clust < m_num_walls; clust++) {
 //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //   clustered_clouds.push_back(cloud);
 //  }
 //
 //
 //
 //
 //  int clust_idx;
 //  //For each point in the mesh get the plane that is closest to it.
 //  for (size_t i = 0; i < m_points_wrapped.size(); i++) {
 //
 //    int closest_plane_idx=-1;
 //    double distance=999999999;
 //
 //    for (size_t plane_idx = 0; plane_idx < planes.size(); plane_idx++) {
 //      pcl::PointXYZ p;
 //      p.x=m_points_wrapped[i][0];
 //      p.y=m_points_wrapped[i][1];
 //      p.z=m_points_wrapped[i][2];
 //      double cur_dist=pcl::pointToPlaneDistanceSigned (p , planes[plane_idx].coef.values[0],
 //                                                           planes[plane_idx].coef.values[1],
 //                                                           planes[plane_idx].coef.values[2],
 //                                                           planes[plane_idx].coef.values[3]);
 //
 //      cur_dist=std::fabs(cur_dist);
 //      if (cur_dist < distance){
 //        distance= cur_dist;
 //        closest_plane_idx=plane_idx;
 //      }
 //    }
 //
 //
 //    pcl::PointXYZ pt;
 //    pt.x=m_points_wrapped[i][0];
 //    pt.y=m_points_wrapped[i][1];
 //    pt.z=m_points_wrapped[i][2];
 //
 //
 //    clust_idx=planes[closest_plane_idx].index_cluster;
 //    clustered_clouds[clust_idx]->points.push_back(pt);
 //    clustered_points_indices[clust_idx].push_back(i);
 //  }
 //
 //  std::cout << "finished clutering points" << std::endl;
 //
 //  for (size_t i = 0; i < m_num_walls; i++) {
 //    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
 //    // viewer.showCloud (clustered_clouds[i]);
 //    // while (!viewer.wasStopped ())
 //    // {
 //    // }
 //  }
 //
 //
 //  //-----------------------
 //
 //
 //
 //
 //  //NOW we have the clutered clouds
 //
 //
 //  //Decimate the clouds
 //  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
 //  for (size_t clust = 0; clust < cluster_count; clust++) {
 //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //    clustered_clouds_ds.push_back(cloud);
 //  }
 //  for (size_t clust = 0; clust < cluster_count; clust++) {
 //    pcl::VoxelGrid<pcl::PointXYZ> ds;  //create downsampling filter
 //    ds.setInputCloud (clustered_clouds[clust]);
 //    ds.setLeafSize (0.1, 0.1, 0.1);
 //    ds.filter (*clustered_clouds_ds[clust]);
 //  }
 //
 //
 //
 //
 //  //vector of planes that will be ftted to the clouds
 //  planes.clear();
 //  planes.resize(cluster_count);
 //  for (size_t i = 0; i < cluster_count; i++) {
 //     planes[i].coef.values.resize(4);
 //     planes[i].index_cluster=i;
 //   }
 //
 //
 //  std::cout << "segmenting" << std::endl;
 //
 //  m_inliers_vec.clear();
 //
 //  #pragma omp parallel for
 //  for (size_t clust = 0; clust < cluster_count; clust++) {
 //    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
 //    pcl::SACSegmentation<pcl::PointXYZ> seg2;
 //
 //    //  seg.setOptimizeCoefficients (true);
 //
 //    seg2.setModelType (pcl::SACMODEL_PLANE);
 //    seg2.setMethodType (pcl::SAC_RANSAC);
 //    //  seg.setDistanceThreshold (0.02);
 //    seg2.setDistanceThreshold (0.05);
 //   // seg.setDistanceThreshold (0.007);
 //
 //    seg2.setInputCloud (clustered_clouds[clust]);
 //    seg2.segment (*inliers2, (planes[clust].coef));
 //
 //    if (inliers2->indices.size () == 0)
 //    {
 //      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
 //    }
 //
 //    m_inliers_vec.push_back(inliers2);
 //
 //
 //    std::cerr << "Model coefficients: " << planes[clust].coef.values[0] << " "
 //                                         << planes[clust].coef.values[1] << " "
 //                                         << planes[clust].coef.values[2] << " "
 //                                         << planes[clust].coef.values[3] << std::endl;
 // }
 //
 //
 // //Add the normal as an eigen vector for easier calculations
 // for (size_t i = 0; i < planes.size(); i++) {
 //   planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
 // }
 //
 //
 // //Fix normals of planes so that they point towards the center
 // //Dot product between the (point on a plane- center) and the normal (if it's negative then flip the normal)
 // //WATCHOUT I SET Z and Y to 0 because if I set x and y Imight not get a solution since the plane might be paralel to Z axis.
 // //Needs a more reobust way of finding points to account for parallel planes
 // for (size_t i = 0; i < planes.size(); i++) {
 //   Eigen::Vector3f dir((- (planes[i].coef.values[3] / planes[i].coef.values[0]) ) - m_center[0],
 //                       - m_center[1],
 //                       - m_center[2]);
 //
 //   double dot= planes[i].normal.dot(dir);
 //
 //   if (dot<0.0){
 //     std::for_each(planes[i].coef.values.begin(), planes[i].coef.values.end(), [](float& d) { d=-d;});
 //     planes[i].normal = Eigen::Vector3f::Map(planes[i].coef.values.data(), 3);
 //   }
 // }
 //
 //
 //
 // //get the angles of the planes with respect to the x axis
 // for (size_t i = 0; i < planes.size(); i++) {
 //     Eigen::Vector2f normal = Eigen::Vector2f::Map(planes[i].coef.values.data(), 2);
 //     Eigen::Vector2f x_axis (1.0, 0.0);
 //
 //
 //     float dot = normal.dot(x_axis);      //dot produt
 //     float cross = normal.x()*x_axis.y() - x_axis.x()*normal.y(); //cross, determinant
 //     double angle = atan2(cross, dot) ;  // atan2(y, x) or atan2(sin, cos)
 //
 //     angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);
 //
 //     planes[i].angle=angle;
 // }
 // std::sort(planes.begin(), planes.end(), by_angle());
 //
 //
 //
 //
 // //Writing the original walls in the model
 // // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds_ds;
 // clustered_clouds_original.clear();
 // for (size_t clust = 0; clust < cluster_count; clust++) {
 //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //   clustered_clouds_original.push_back(cloud);
 // }
 // for (size_t clust = 0; clust < cluster_count; clust++) {
 //   clustered_clouds_original[clust]->width    = clustered_clouds[clust]->points.size();
 //   clustered_clouds_original[clust]->height   = 1;
 //   clustered_clouds_original[clust]->is_dense = false;
 //   clustered_clouds_original[clust]->points.resize (clustered_clouds[clust]->points.size());
 //
 //   for (size_t i = 0; i < clustered_clouds[clust]->points.size(); i++){
 //     clustered_clouds_original[clust]->points[i].x =  clustered_clouds[clust]->points[i].x;
 //     clustered_clouds_original[clust]->points[i].y =  clustered_clouds[clust]->points[i].y;
 //     clustered_clouds_original[clust]->points[i].z =  clustered_clouds[clust]->points[i].z;
 //   }
 //
 // }
 //
 //
 //
 //
 //
 //
 //
 //
 //
 // // go through the planes and calculate the rotation translation matrix to map them
 // m_plane_centers.resize(planes.size());
 // #pragma omp parallel for
 // for (size_t i = 0; i < planes.size(); i++) {
 //   //TODO::MAKE ALL this code use to use eigen, clean it
 //
 //   vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
 //   planeSource->SetCenter(0.0, 0.0, 0.0);
 //   planeSource->SetNormal( planes[i].coef.values[0],
 //                           planes[i].coef.values[1],
 //                           planes[i].coef.values[2]);
 //   planeSource->Push(-planes[i].coef.values[3]);
 //   planeSource->Update();
 //
 //   double center_vtk[3];
 //   planeSource->GetCenter(center_vtk);
 //   m_plane_centers[i]=row_type {center_vtk[0],center_vtk[1],center_vtk[2]};
 //   // Eigen::Vector3f plane_center = Eigen::Vector3f::Map(center_vtk, 3);
 //   // plane_center[2]=0.0;
 //
 //   Eigen::Vector3f plane_center;
 //   plane_center[0]=center_vtk[0];
 //   plane_center[1]=center_vtk[1];
 //   plane_center[2]=0.0;
 //
 //
 //   // double offset= 0.03*i;  //TODO REMOVE: Each plane gets closer and closer
 //   // double offset= 0.05*i;  //TODO REMOVE: Each plane gets closer and closer
 //   double offset= 0.00*i;  //TODO REMOVE: Each plane gets closer and closer
 //
 //
 //   row_type end_point(3);
 //   end_point[0]= planes[i].angle * m_circumference - offset; //I don't know why it needs to be summed and not -
 //   end_point[1]= 0.0;
 //   end_point[2]= 0.0;
 //
 //
 //   //TEST: Second way of calculating the end point
 //   // double angle_wall=360.0/(double)m_num_walls;
 //   // angle_wall= angle_wall *M_PI/180.0;
 //   // double side= sqrt (m_radius*m_radius +m_radius*m_radius -2*m_radius*m_radius* cos (angle_wall));
 //   // // pos=pos/2;
 //   // double pos= side*i;
 //   // pos=pos- (side/2);
 //   // end_point[0]=pos;
 //   //-----------------
 //
 //   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
 //   Eigen::Affine3f transform_center = Eigen::Affine3f::Identity();
 //
 //   // Define a translation of 2.5 meters on the x axis.
 //   transform_2.translation() << end_point[0], end_point[1], end_point[2];
 //   transform_center.translation() << -plane_center[0], -plane_center[1], -plane_center[2];
 //
 //
 //   double r = (90 * M_PI /180) + interpolate (planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
 //   transform_2.rotate (Eigen::AngleAxisf (r, Eigen::Vector3f::UnitZ()));
 //
 //
 //   std::cout << transform_2.matrix() << std::endl;
 //   int clust_idx= planes[i].index_cluster;
 //
 //   pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform_center);
 //   pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform_2);
 //
 // }
 //
 //
 //
 //
 // //Move the walls closer together
 // matrix_type min_max(cluster_count);
 // // for (size_t clust = 0; clust < cluster_count; clust++) {
 // //   //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
 // //   double min_x_cur=std::numeric_limits<double>::max();;
 // //   double max_x_cur=std::numeric_limits<double>::min();;
 // //   for (size_t i_idx = 0; i_idx < m_inliers_vec[clust]->indices.size (); i_idx++) {
 // //     if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x > max_x_cur  ){
 // //       max_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
 // //     }
 // //     if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x < min_x_cur  ){
 // //       min_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
 // //     }
 // //
 // //   }
 // //   // std::cout << "cluster " << clust  << std::endl;
 // //   // std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;
 // //
 // //   min_max[clust]=row_type {min_x_cur, max_x_cur};  //there are now unordered
 // // }
 //
 //
 // for (size_t clust = 0; clust < cluster_count; clust++) {
 //   //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
 //   double min_x_cur=std::numeric_limits<double>::max();
 //   double max_x_cur=std::numeric_limits<double>::min();
 //   for (size_t i_idx = 0; i_idx < clustered_clouds[clust]->points.size (); i_idx++) {
 //     if ( clustered_clouds[clust]->points[i_idx].x > max_x_cur  ){
 //       max_x_cur=clustered_clouds[clust]->points[i_idx].x;
 //     }
 //     if ( clustered_clouds[clust]->points[i_idx].x < min_x_cur  ){
 //       min_x_cur=clustered_clouds[clust]->points[i_idx].x;
 //     }
 //
 //   }
 //   // std::cout << "cluster " << clust  << std::endl;
 //   // std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;
 //
 //   min_max[clust]=row_type {min_x_cur, max_x_cur};  //there are now unordered
 // }
 //
 //
 // //Now that we have the min and max, move them
 // std::cout << "movements" << std::endl;
 // row_type movements(cluster_count, 0.0);
 // for (size_t i = 0; i < planes.size(); i++) {
 //   int clust_idx= planes[i].index_cluster;
 //   if (i==0){
 //     movements[clust_idx]= 0.0 - min_max[clust_idx][0];
 //   }else{
 //     movements[clust_idx]=0.0;
 //     movements[clust_idx]+= movements[planes[i-1].index_cluster];
 //     movements[clust_idx]+= (min_max[planes[i-1].index_cluster][1] - min_max[clust_idx][0] );
 //   }
 //   std::cout << "clust index " << clust_idx << std::endl;
 //   std::cout << "movemnt: " << movements[clust_idx] << std::endl;
 // }
 //
 // std::cout << "move them given the min and max" << std::endl;
 // for (size_t i = 0; i < planes.size(); i++) {
 //   int clust_idx= planes[i].index_cluster;
 //
 //
 //
 //   std::cout << "plane: " << i << std::endl;
 //   std::cout << "min max: " << min_max[clust_idx][0] << " " << min_max[clust_idx][1] << std::endl;
 //
 //
 //
 //   //Move it
 //   // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
 //   // transform.translation() << min_x_cur - max_x_prev, 0.0, 0.0;
 //   // pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform);
 //   for (size_t p_idx = 0; p_idx < clustered_clouds[clust_idx]->points.size(); p_idx++) {
 //     clustered_clouds[clust_idx]->points[p_idx].x = clustered_clouds[clust_idx]->points[p_idx].x + movements[clust_idx];
 //   }
 //
 // }
 //
 //
 // //showing stuff
 // // for (size_t i = 0; i < planes.size(); i++) {
 // //   int clust_idx= planes[i].index_cluster;
 // //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
 // //    viewer.showCloud (clustered_clouds[clust_idx]);
 // //    while (!viewer.wasStopped ())
 // //    {
 // //    }
 // // }
 //
 //
 //
 //
 // std::cout << "wiritng to unwrapped points" << std::endl;
 //
 // //write the unwrapped points
 // m_points_unwrapped.resize(m_points_wrapped.size());
 // for (size_t i = 0; i < m_points_unwrapped.size(); i++) {
 //   m_points_unwrapped[i].resize(3);
 // }
 //
 // for (size_t clust = 0; clust < cluster_count; clust++) {
 //   int idx=0;
 //
 //   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
 //   kdtree.setInputCloud (clustered_clouds[clust]);
 //
 //   for (size_t i = 0; i < clustered_clouds[clust]->points.size(); i++) {
 //
 //     //we have a point and an index to insert it to, move the index until it winds the fisrt label
 //    //  while(labels.at<int>(idx,0)!=clust){
 //    //    idx++;
 //    //  }
 //
 //    idx= clustered_points_indices[clust][i];
 //
 //     m_points_unwrapped[idx][0]= clustered_clouds[clust]->points[i].x;
 //     m_points_unwrapped[idx][1]= clustered_clouds[clust]->points[i].y;
 //     m_points_unwrapped[idx][2]= clustered_clouds[clust]->points[i].z;
 //
 //
 //     //Need to check the nearest neightbours to fix the y (ditance to the plane)
 //     pcl::PointXYZ searchPoint;
 //     searchPoint=clustered_clouds[clust]->points[i];
 //
 //
 //     if (m_deform_walls){
 //       int K = 50;
 //       // double radius=0.1;
 //       double radius=0.05;
 //       //  double radius=0.16;
 //
 //       std::vector<int> pointIdxNKNSearch(K);
 //       std::vector<float> pointNKNSquaredDistance(K);
 //
 //       double avg_dist=0.0;
 //
 //       if (  kdtree.radiusSearch (searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
 //       {
 //        for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p){
 //           // //average the dist of the K nearest neghbours
 //           avg_dist+=clustered_clouds[clust]->points[ pointIdxNKNSearch[p] ].y;
 //         }
 //       }
 //
 //       avg_dist=avg_dist/pointIdxNKNSearch.size ();
 //       m_points_unwrapped[idx][1]=m_points_unwrapped[idx][1]-avg_dist;
 //     }
 //
 //
 //
 //
 //     //clap the distance
 //     // if ( fabs(m_points_unwrapped[idx][1]) > 0.1 ){
 //     //   m_points_unwrapped[idx][1]=0.0;
 //     //   clustered_clouds[clust]->points[i].y=0.0;
 //     // }
 //
 //   }
 // }
 //









  //TODO calculate new normals for the unwrapped mesh
}



std::vector<double> Mesh::computeAngles(matrix_type points){


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

std::vector<double> Mesh::computeDistancesToRadius(matrix_type points, double radius){
  int num_points=points.size();
  std::vector<double> dist;
  dist.resize(num_points);

  //Calculate first the distance to center.
  for (size_t i = 0; i < num_points; i++) {
    dist[i]= sqrt( points[i][0]*points[i][0]  + points[i][1]*points[i][1] );
    dist[i]= radius- dist[i];
  }

  return dist;
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

//TODO clean the code
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

void Mesh::compute_depth_rgb_colors(){
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


  //cap th depth at a certain value both up and down
  for (size_t i = 0; i < num_points; i++) {
    // depth[i]=interpolate(depth[i], 0.0 , 255.0 , m_low_cap_depth_color,  m_high_cap_depth_color);
    if (depth[i]>m_high_cap_depth_color){
      depth[i]=m_high_cap_depth_color;
    }
    if (depth[i]<m_low_cap_depth_color){
      depth[i]=m_low_cap_depth_color;
    }
    depth[i]=interpolate(depth[i], m_low_cap_depth_color , m_high_cap_depth_color , 0.0,  255.0);

  }

  m_colors_active->Reset();
  m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active->SetNumberOfComponents(3);
  m_colors_active->SetName("depth_rgb");



  cv::Mat depth_cv(depth.size(), 1,  CV_8UC1);

  //copy the pixels into a OpencvMat
  for (size_t i = 0; i < depth.size(); i++) {
    depth_cv.at<uchar>(i,0)=depth[i];
  }

  //colormap the OpencvMat
  cv::Mat depth_colormaped_cv;
  cv::applyColorMap(depth_cv, depth_colormaped_cv, cv::COLORMAP_JET);

  //Copy colors bright back into a unsigned char arrays
  for (int i = 0; i < depth_colormaped_cv.cols; i++) {
    for (int j = 0; j < depth_colormaped_cv.rows; j++) {
        cv::Vec3b intensity = depth_colormaped_cv.at<cv::Vec3b>(j, i);
        double color_pixel[3];
        color_pixel[0]= intensity.val[0];
        color_pixel[1]= intensity.val[1];
        color_pixel[2]= intensity.val[2];
        m_colors_active->InsertNextTuple(color_pixel);
    }
  }



  // std::cout << "starting to create colors" << std::endl;
  // for (size_t i = 0; i < num_points; i++) {
  //   //colors->InsertNextTuple3(angles[i]*255.0,0,0);
  //   m_colors_active->InsertNextTuple3(depth[i],depth[i],depth[i]);
  //   //m_points_unwrapped.InsertNextPoint( angles[i] *circumference,distances_from_radius[i],point[2])
  // }
  std::cout << "finishing to create colors" << std::endl;
}


void Mesh::compute_depth_defects_colors(){


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

    //sticking out
    if (m_points_unwrapped[i][1]>0.0){
      depth[i]=interpolate(this->m_points_unwrapped[i][1], 0.0, max_dist, 0.5, 1.0);
      // depth[i]=1.0;
    }else{ //sunk in
      depth[i]=interpolate(this->m_points_unwrapped[i][1], 0.0, min_dist, 0.5, 0.0);
      // depth[i]=0.0;
    }
  }

  m_colors_active->Reset();
  m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  m_colors_active->SetNumberOfComponents(3);
  m_colors_active->SetName("depth");


  // std::cout << "starting to create colors" << std::endl;
  // for (size_t i = 0; i < num_points; i++) {
  //   //colors->InsertNextTuple3(angles[i]*255.0,0,0);
  //
  //   // std::cout << "depth is    "  << depth[i] << '\n';
  //
  //
  //   double r=125.0,g=125.0,b=125.0;
  //   // double m_above_tresh=0.6;
  //   // double m_below_tresh=0.4;
  //
  //   //if it's sticking out too much then set it to red
  //   if (depth[i]> m_above_tresh){
  //     // double red_start_val=depth[i];
  //     // r=interpolate(depth[i], thresh_red, 0, red_start_val  , 255);
  //     r=255.0;
  //     g=0.0;
  //     b=0.0;
  //     // r=interpolate(depth[i], m_above_tresh, 1.0, 125.0  , 255);
  //     // g=interpolate(depth[i], m_above_tresh, 1.0, 125.0  , 0);
  //     // b=interpolate(depth[i], m_above_tresh, 1.0, 125.0  , 0);
  //   }
  //
  //   //if it's sunk in too much then set it to red
  //   else if (depth[i]< m_below_tresh){
  //     // double red_start_val=depth[i];
  //     // r=interpolate(depth[i], thresh_red, 0, red_start_val  , 255);
  //     r=0.0;
  //     g=0.0;
  //     b=255.0;
  //     // r=interpolate(depth[i], m_below_tresh, 0.0, 125.0  , 0);
  //     // g=interpolate(depth[i], m_below_tresh, 0.0, 125.0  , 0);
  //     // b=interpolate(depth[i], m_below_tresh, 0.0, 125.0  , 255);
  //
  //   }
  //
  //   //otherwise it is just green
  //   // std::cout << "vals is " << r << " " << g << " " << b << '\n';
  //
  //
  //   m_colors_active->InsertNextTuple3(r,g,b);
  //   // m_colors_active->InsertNextTuple3(depth[i]*255,depth[i]*255,depth[i]*255);
  //   //m_points_unwrapped.InsertNextPoint( angles[i] *circumference,distances_from_radius[i],point[2])
  // }
  // std::cout << "finishing to create colors" << std::endl;



  //write points to and cells to mesh
  vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  points_active=vector_to_vtk(m_points_unwrapped);
  m_wall->SetPoints(points_active);
  m_wall->SetPolys(m_cells_unwrapped);
  vtkSmartPointer<vtkPolyData> mesh = m_wall;


  matrix_type_i neighbours_ids_all;
  precompute_all_neighbours(mesh, neighbours_ids_all);

  std::cout << "finishe precmputing" << '\n';

  //Assign each point as being red or blue
  std::vector < row_type_i> connected_components; //first index is the connected component and the second is the indexes poiting at the points that are inside the component
  std::vector<bool> already_in_component(m_points_unwrapped.size(),false);

  for (size_t p_idx = 0; p_idx < m_points_unwrapped.size(); p_idx++) {
    if(!already_in_component[p_idx] && depth[p_idx]> m_above_tresh){

      //It will start a new component
      row_type_i component_ids;
      std::queue<int> expansion_front;
      expansion_front.push(p_idx);
      component_ids.push_back(p_idx);
      already_in_component[p_idx]=true;
      // std::cout << "----------connected from point " << i << '\n';
      // utils::tick();
      while (!expansion_front.empty()){

        row_type_i connected_vertices;
        int expand_idx=expansion_front.front();
        expansion_front.pop();
        connected_vertices=neighbours_ids_all[expand_idx];

        //if the connected vertices are not in neighbours and also are within the radius, add to expansion front
        for (size_t c_idx = 0; c_idx < connected_vertices.size(); c_idx++) {
          bool will_add_to_component=true;
          //check if its already in the another component and if its also red
          if ( depth[ connected_vertices[c_idx]] < m_above_tresh || already_in_component[connected_vertices[c_idx]] ){
            will_add_to_component=false;
          }

          //Check if its not part already of this component
          // if (will_add_to_component){
          //   for (size_t n_idx = 0; n_idx < component_ids.size(); n_idx++) {
          //     if (connected_vertices[c_idx]==component_ids[n_idx]){
          //       will_add_to_component=false;
          //     }
          //   }
          // }


          if (will_add_to_component){
              expansion_front.push(connected_vertices[c_idx]);
              component_ids.push_back(connected_vertices[c_idx]);
              already_in_component[connected_vertices[c_idx]]=true;
          }

        }
      }
      connected_components.push_back(component_ids);


    }
  }

  std::cout << "nr of connected_components " << connected_components.size() << '\n';


  int biggest_component=0;
  for (size_t i = 0; i < connected_components.size(); i++) {
    if (connected_components[i].size()>biggest_component){
      biggest_component=connected_components[i].size();
    }
  }

  std::cout << "biggest component is " << biggest_component << '\n';


  //Default gray colors
  row_type_i points_with_defects;
  for (size_t i = 0; i < num_points; i++) {
    points_with_defects.push_back(0);
  }

  // int component_thresh=2500;
  // int component_thresh=4500;
  int component_thresh=9500;

  //Colorize the big components as red
  for (size_t i = 0; i < connected_components.size(); i++) {
    //if the component is big enough color it blue otherwise gray
    if (connected_components[i].size()>component_thresh){
      for (size_t c_idx = 0; c_idx < connected_components[i].size(); c_idx++) {
        int p_idx= connected_components[i][c_idx];
        points_with_defects[p_idx]=1;
      }
      // m_colors_active->InsertNextTuple3(255,0,0);
    }
  }

  for (size_t i = 0; i < num_points; i++) {
    if (points_with_defects[i]==1) {
       m_colors_active->InsertNextTuple3(255,0,0);
    }else{
       m_colors_active->InsertNextTuple3(125,125,125);
    }
  }


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
  // m_is_cylindrical = false;
  m_points_wrapped.clear();
}
