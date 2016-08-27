#include "Model.h"
#include <iostream>

Model::Model():
  m_radius(0.0),
  m_circumference(0.0),
  m_is_unwrapped(false),
  m_selecting_defects(false),
  m_selecting_grid(false),
  m_wall(vtkSmartPointer<vtkPolyData>::New()),
  // m_cells(vtkSmartPointer<vtkCellArray>::New()),
  m_colors_original(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  m_colors_active(vtkSmartPointer<vtkUnsignedCharArray>::New()),
  m_deleted_streached_trigs(false),
  m_draw_grid_active(true),
  m_draw_grid_inactive(false),
  m_points_unwrapped_full_cloud(new pcl::PointCloud<pcl::PointXYZ>),

  m_cells_wrapped(vtkSmartPointer<vtkCellArray>::New()),
  m_cells_unwrapped(vtkSmartPointer<vtkCellArray>::New())
{


}


void Model::set_mesh(vtkSmartPointer<vtkPolyData> mesh){
  clear();
  m_wall=mesh;
  center_mesh();
  read_info();
  //scale_mesh();  //TODO: REMOVE this one because it was only used so as to better see the fitted planes
}

void Model::set_texture(vtkSmartPointer<vtkTexture> texture){
  m_full_texture=texture;
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
  m_grid.clear();
  m_grid_cells_active.clear();
}


void Model::read_info(){
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points=m_wall->GetPoints();
  m_points_wrapped= vtk_to_vector(points);


  m_points_wrapped_ds=compute_decimated(m_points_wrapped);
  m_cells_wrapped    =this->m_wall->GetPolys();
  m_colors_original  = get_colors();
  m_colors_active  = get_colors();
  m_radius           = estimate_radius(m_points_wrapped_ds);
  m_circumference    =2*M_PI*m_radius;   //In the case of a non cylindrical one, the circumference is the perimeter of all the walls

  // see law of cosines
  double angle_wall=360.0/(double)m_num_walls;
  angle_wall= angle_wall *M_PI/180.0;
  m_circumference =m_num_walls * sqrt (m_radius*m_radius +m_radius*m_radius -2*m_radius*m_radius* cos (angle_wall));

  std::cout << "circumefrence is " << m_circumference << std::endl;


  vtkSmartPointer<vtkDataArray> vtk_normals = m_wall->GetPointData()->GetNormals();
  if(vtk_normals){
    std::cout << "the polydata has normals" << std::endl;
  }else{
    std::cout << "the polydata does not have normals" << std::endl;
  }
  m_normals=vtk_normal_tcoords_to_vector(vtk_normals);

  m_bounds=m_wall->GetBounds();



  // //GEt texture coordinates
  // vtkSmartPointer<vtkDataArray> vtk_tcoords = m_wall->GetPointData()->GetTCoords();
  // if(vtk_tcoords){
  //   std::cout << "the polydata has tcoords" << std::endl;
  // }else{
  //   std::cout << "the polydata does not have tcoords" << std::endl;
  // }
  // m_tcoords_wrapped=vtk_normal_tcoords_to_vector(vtk_tcoords);





  std::cout << "model::readinfo: num points= " <<m_points_wrapped.size() << std::endl;
  std::cout << "model::readinfo: we have nr cells: " <<  m_cells_wrapped->GetSize()  << std::endl;
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

  int further_num=50;
  std::vector< pcl::PointXYZ > further_points(m_points_wrapped_ds->points.begin(), m_points_wrapped_ds->points.begin() + further_num);


  //Get the median of those distance as the radius
  row_type distances(further_num);
  for (size_t i = 0; i < further_points.size(); i++) {
    float dist= pcl::geometry::distance(further_points[i], pcl::PointXYZ (0.0, 0.0, further_points[i].z));
    distances[i]=dist;
  }

  radius= median(distances);

  std::cout << "radius is " << radius << std::endl;

  std::cout << "finish estimating radius" << std::endl;
  return radius;

}


vtkSmartPointer<vtkPolyData> Model::get_mesh(){
  std::cout << "model::updating mesh" << std::endl;

  write_points_to_mesh();
  return m_wall;

}

void Model::write_points_to_mesh(){
  std::cout << "writig to mehs!!!!!!!" << std::endl;
  if (m_points_wrapped.empty()){ //m_wall has not been set yet
    return;
  }

  //Set points
  vtkSmartPointer<vtkPoints> points_active = vtkSmartPointer<vtkPoints>::New() ;
  if (this->m_is_unwrapped){
    points_active=vector_to_vtk(this->m_points_unwrapped);
  }else{
    points_active=vector_to_vtk(this->m_points_wrapped);
  }

  std::cout << "setting points" << std::endl;

  m_wall->SetPoints(points_active);

  std::cout << "setting colors" << std::endl;
  m_wall->GetPointData()->SetScalars(this->m_colors_active);

  std::cout << "deleting streched" << std::endl;
  if (this->m_is_unwrapped){
    delete_streched_trigs();
  }


  std::cout << "setting cells" << std::endl;
  if (m_is_unwrapped){
    std::cout << "wiritng unwrapped cels!!!!!!!!!!!!11" << std::endl;
      std::cout << "nr os cells: " << m_cells_unwrapped->GetSize() << std::endl;
    m_wall->SetPolys(this->m_cells_unwrapped);
  }else{
    std::cout << "wiritng wrapped cels!!!!!!!!!!!!11" << std::endl;
    std::cout << "nr os cells: " << m_cells_wrapped->GetSize() << std::endl;
    m_wall->SetPolys(m_cells_wrapped);
  }


  this->m_bounds=m_wall->GetBounds();

  //TODO: we should recalculate new normals, insted we should have normal_wrapped and normals unwrapped
  vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();


  #if VTK_MAJOR_VERSION <= 5
    normals_alg->SetInputConnection(m_wall->GetProducerPort());
  #else
    normals_alg->SetInputData(m_wall);
  #endif

  normals_alg->Update();

  m_wall=normals_alg->GetOutput();

}


void Model::delete_streched_trigs(){
  //get all the cells and calculate the perimeter. If the perimeter excedes a certain value, delete the cell (poly)

  if (m_deleted_streached_trigs)
    return;

  m_deleted_streached_trigs=true;

  // std::cout << "delete streached trigs" << std::endl;
  m_wall->BuildLinks();

  vtkIdType n_pts=-1,*pts;
  m_wall->GetPolys()->InitTraversal();

  // m_streched_polys.resize(m_wall->GetPolys()->GetNumberOfCells());

  int counter=0;
  for (size_t i = 0; i < m_wall->GetPolys()->GetNumberOfCells(); i++) {
    m_wall->GetPolys()->GetNextCell(n_pts,pts);
    // m_streched_polys[i]=row_type {pts[0],pts[1],pts[2]};

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
    double thresh=0.1;

    if (  dist(trig[0], trig[1])  > thresh || dist(trig[0], trig[2]) >thresh  || dist(trig[2], trig[1]) >thresh ){
      // std::cout << "delete cell" << i << std::endl;
      m_wall->DeleteCell(i);
      counter++;
    }

  }

  m_wall->RemoveDeletedCells();

  // //clean and read again but now into points unwrapped
  // vtkSmartPointer<vtkCleanPolyData> clean = vtkSmartPointer<vtkCleanPolyData>::New();
  // clean->SetInputData(m_wall);
  // clean->Update();

  // this->m_wall=clean->GetOutput();
  // read_info();

  //Read inf again but now into points unwrapped
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points=m_wall->GetPoints();
  m_points_unwrapped= vtk_to_vector(points);
  m_cells_unwrapped            =m_wall->GetPolys();


  // std::cout << "Deletet nr cells: " << counter << std::endl;
  // std::cout << "after deleting streched trings we have nr points: " <<  m_points_unwrapped.size()  << std::endl;
  // std::cout << "after deleting streched trings we have nr cells: " <<  m_cells_unwrapped->GetSize()  << std::endl;

}


void Model::add_streched_trigs(){


  // for (size_t i = 0; i < m_streched_polys; i++) {
  //   pointElems->InsertNextCell(3);
  // }

}



double Model::dist(row_type vec1, row_type vec2){
  double dist=0.0;
  for (size_t i = 0; i < vec1.size(); i++) {
    dist+= std::pow (vec1[i] - vec2[i],2);
  }
  dist=sqrt(dist);
}


void Model::blur_normals(){

  std::cout << "blurring the normals" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZ>);

  cloud_normals->width    = m_normals.size();
  cloud_normals->height   = 1;
  cloud_normals->is_dense = false;
  cloud_normals->points.resize (m_normals.size());

  for (size_t i = 0; i < m_points_wrapped.size(); i++){
    cloud_normals->points[i].x = m_normals[i][0];
    cloud_normals->points[i].y = m_normals[i][1];
    cloud_normals->points[i].z = m_normals[i][2];
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
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;


  for (size_t i = 0; i < cloud_points->points.size(); i++) {
    if (  kdtree.radiusSearch (cloud_points->points[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
     row_type normal(3, 0.0);
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

      m_normals[i][0] = normal[0];
      m_normals[i][1] = normal[1];
      m_normals[i][2] = normal[2];
    }
  }



  std::cout << "finished blurring the normals" << std::endl;

}


void Model::compute_unwrap2(){
  std::cout << "computing unwrap 2" << std::endl;

  //Blur the normals so as to better detect the walls
  blur_normals();


  //Cluster the normals of the points.
  cv::Mat samples(m_normals.size(), 3, CV_32F);
  for( int y = 0; y < samples.rows; y++ ){
    for( int x = 0; x < samples.cols; x++ ){
      samples.at<float>(y,x)=m_normals[y][x];
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
    // int num_items = std::count(labels_vec.begin(), labels_vec.end(), clust);
    // cloud->points.resize(num_items);
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


   m_ransac_centers.resize(cluster_count);
   std::cout << "segmenting" << std::endl;
   //segmtn each cloud and gt the coefficients of the planes fitted
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

     seg.setInputCloud (clustered_clouds[clust]);
     seg.segment (*inliers, (planes[clust].coef));

     if (inliers->indices.size () == 0)
     {
       PCL_ERROR ("Could not estimate a planar model for the given dataset.");
     }

     m_inliers_vec.push_back(inliers);

     //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
    //  double min=999999;
    //  double max=-999999;
    //  for (size_t i_idx = 0; i_idx < inliers->indices.size (); i_idx++) {
    //    if ( clustered_clouds_ds[clust]->points[inliers->indices[i_idx]].x > max  ){
    //      max=clustered_clouds_ds[clust]->points[inliers->indices[i_idx]].x;
    //    }
    //    if ( clustered_clouds_ds[clust]->points[inliers->indices[i_idx]].x < min  ){
    //      min=clustered_clouds_ds[clust]->points[inliers->indices[i_idx]].x;
    //    }
     //
    //  }
    //  std::cout << "cluster " << clust  << std::endl;
    //  std::cout << "min max " <<  min << " " << max << clust  << std::endl;


    //Get the centers of the inliers
    // row_type center(3, 0.0);
    //
    // for (size_t i = 0; i < inliers->indices.size (); i++) {
    //   center[0]+=clustered_clouds[clust]->points[inliers->indices[i]].x;
    //   center[1]+=clustered_clouds[clust]->points[inliers->indices[i]].y;
    //   center[2]+=clustered_clouds[clust]->points[inliers->indices[i]].z;
    // }
    // center[0]=center[0]/inliers->indices.size ();
    // center[1]=center[1]/inliers->indices.size ();
    // center[2]=center[2]/inliers->indices.size ();
    // m_ransac_centers[clust]=center;





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



  //Get the walls min and max
  // m_walls_wrapped_bounds.resize(cluster_count);
  // for (size_t i = 0; i < cluster_count; i++) {
  //   //Move rotate the cluster by the angle
  //   int clust_idx= planes[i].index_cluster;
  //   Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  //   double r = (90 * M_PI /180) + interpolate (planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
  //   transform.rotate (Eigen::AngleAxisf (r, Eigen::Vector3f::UnitZ()));
  //   pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform);
  //
  //   //Get bounding box
  //   pcl::PointXYZ p_min;
  //   pcl::PointXYZ p_max;
  //   pcl::getMinMax3D (*(clustered_clouds[clust_idx]), p_min, p_max);
  //   row_type bounds(6);
  //
  //   bounds[0]=p_min.x;
  //   bounds[1]=p_max.x;
  //   bounds[2]=p_min.y;
  //   bounds[3]=p_max.y;
  //   bounds[4]=p_min.z;
  //   bounds[5]=p_max.z;
  //   m_walls_wrapped_bounds[clust_idx]=bounds;
  //
  //   vtkSmartPointer<vtkOutlineSource> outlineSource = vtkSmartPointer<vtkOutlineSource>::New();
  //   outlineSource->SetBounds(bounds.data());
  //   outlineSource->Update();
  //
  //
  //   //Rotate it back
  //   pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform.inverse());
  //   vtkSmartPointer<vtkTransform> rotate_bb = vtkSmartPointer<vtkTransform>::New();
  //   rotate_bb->RotateZ(-r*180/M_PI);
  //
  //   vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  //   transformFilter->SetInputConnection(outlineSource->GetOutputPort());
  //   transformFilter->SetTransform(rotate_bb);
  //   transformFilter->Update();
  // }



  //Attempt 2 at getting min and max //orientated bounding boxes
  // for (size_t clust = 0; clust < cluster_count; clust++) {
  //
  //
  //   // compute principal direction
  //   Eigen::Vector4f centroid;
  //   pcl::compute3DCentroid(*(clustered_clouds_ds[clust]), centroid);
  //   Eigen::Matrix3f covariance;
  //   computeCovarianceMatrixNormalized(*(clustered_clouds_ds[clust]), centroid, covariance);
  //   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  //   Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  //   eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));
  //
  //   // move the points to the that reference frame
  //   Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  //   p2w.block<3,3>(0,0) = eigDx.transpose();
  //   p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
  //   pcl::PointCloud<pcl::PointXYZ> cPoints;
  //   pcl::transformPointCloud(*(clustered_clouds_ds[clust]), cPoints, p2w);
  //
  //   pcl::PointXYZ min_pt, max_pt;
  //   pcl::getMinMax3D(cPoints, min_pt, max_pt);
  //   const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
  //
  //   // final transform
  //   const Eigen::Quaternionf qfinal(eigDx);
  //   const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();
  //
  //
  //   //Apply that ration and translation
  //   Eigen::Matrix3f R;
  //   R=qfinal.toRotationMatrix();
  //   Eigen::Vector3f T;
  //   T=tfinal;
  //   Eigen::Matrix4f Trans; // Your Transformation Matrix
  //   Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
  //   Trans.block<3,3>(0,0) = R;
  //   // Trans.rightCols<1>() = T;
  //   Trans.col(3).head<3>()= T;
  //
  //
  //
  //   Eigen::Affine3f affine(Trans);
  //
  //   std::cout << "bb trasnform" << std::endl;
  //   std::cout << affine.matrix() << std::endl;
  //   pcl::transformPoint(min_pt,affine);
  //   pcl::transformPoint(max_pt,affine);
  //
  //
  //   row_type bounds(6);
  //   bounds[0]=min_pt.x;
  //   bounds[1]=max_pt.x;
  //   bounds[2]=min_pt.y;
  //   bounds[3]=max_pt.y;
  //   bounds[4]=min_pt.z;
  //   bounds[5]=max_pt.z;
  //   m_walls_wrapped_bounds[clust]=bounds;
  //
  //   // draw the cloud and the box
  //   // pcl::visualization::PCLVisualizer viewer;
  //   // viewer.addPointCloud((clustered_clouds_ds[clust]));
  //   // viewer.addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);
  //   // viewer.spin();
  //
  // }


  //Attempt 3. just writing the original walls in the model
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


  // // //Move them so that they align better
  // double max_x_prev=0.0;
  // double min_x_prev=0.0;
  // for (size_t i = 0; i < planes.size(); i++) {
  //   int clust_idx= planes[i].index_cluster;
  //
  //   //Get the minimum x of this cluster. Move it so that it coincides with the maximum x of the previous wall
  //   double min_x_cur=99999;
  //   double max_x_cur=-999999;
  //
  //   for (size_t p_idx = 0; p_idx < clustered_clouds[clust_idx]->points.size(); p_idx++) {
  //     if (clustered_clouds[clust_idx]->points[p_idx].x > max_x_cur )
  //       max_x_cur=clustered_clouds[clust_idx]->points[p_idx].x;
  //     if (clustered_clouds[clust_idx]->points[p_idx].x < min_x_cur )
  //       min_x_cur=clustered_clouds[clust_idx]->points[p_idx].x;
  //   }
  //
  //   std::cout << "plane: " << i << std::endl;
  //   std::cout << "min max: " << min_x_cur << " " << max_x_cur << std::endl;
  //
  //   std::cout << "min curr : max_ prev" << min_x_cur << " " << max_x_prev << std::endl;
  //
  //   std::cout << "moving: " << max_x_prev - min_x_cur << std::endl;
  //
  //   //Move it
  //   // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  //   // transform.translation() << min_x_cur - max_x_prev, 0.0, 0.0;
  //   // pcl::transformPointCloud (*(clustered_clouds[clust_idx]), *(clustered_clouds[clust_idx]), transform);
  //   for (size_t p_idx = 0; p_idx < clustered_clouds[clust_idx]->points.size(); p_idx++) {
  //     clustered_clouds[clust_idx]->points[p_idx].x = clustered_clouds[clust_idx]->points[p_idx].x + (max_x_prev - min_x_cur );
  //   }
  //
  //
  //   //After movint we need to update the min max x (brute force way for now)
  //   min_x_cur=99999;
  //   max_x_cur=-999999;
  //   for (size_t p_idx = 0; p_idx < clustered_clouds[clust_idx]->points.size(); p_idx++) {
  //     if (clustered_clouds[clust_idx]->points[p_idx].x > max_x_cur )
  //       max_x_cur=clustered_clouds[clust_idx]->points[p_idx].x;
  //     if (clustered_clouds[clust_idx]->points[p_idx].x < min_x_cur )
  //       min_x_cur=clustered_clouds[clust_idx]->points[p_idx].x;
  //   }
  //
  //   // min_x_cur=min_x_cur+ (max_x_prev - min_x_cur);
  //   // max_x_cur=max_x_cur+ (max_x_prev - min_x_cur);
  //
  //
  //
  //
  //   max_x_prev=max_x_cur;
  //   min_x_prev=min_x_cur;
  // }
  //
  //
  // //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  // //  viewer.showCloud (clustered_clouds_ds[1]);
  // //  while (!viewer.wasStopped ())
  // //  {
  // //  }
  //
  // std::cout << "AFTER MOVING" << std::endl;
  // for (size_t i = 0; i < planes.size(); i++) {
  //   int clust_idx= planes[i].index_cluster;
  //
  //   //Get the minimum x of this cluster. Move it so that it coincides with the maximum x of the previous wall
  //   double min_x_cur=99999;
  //   double max_x_cur=-999999;
  //
  //   for (size_t p_idx = 0; p_idx < clustered_clouds[clust_idx]->points.size(); p_idx++) {
  //     if (clustered_clouds[clust_idx]->points[p_idx].x > max_x_cur )
  //       max_x_cur=clustered_clouds[clust_idx]->points[p_idx].x;
  //     if (clustered_clouds[clust_idx]->points[p_idx].x < min_x_cur )
  //       min_x_cur=clustered_clouds[clust_idx]->points[p_idx].x;
  //   }
  //
  //   std::cout << "plane: " << i << std::endl;
  //   std::cout << "min max: " << min_x_cur << " " << max_x_cur << std::endl;
  // }


  //Second attempt at MOVING
  matrix_type min_max(cluster_count);
  for (size_t clust = 0; clust < cluster_count; clust++) {
    //Go through all the indices and calculate the min max for that cluster_count  (Watchout is is on the ds)
    double min_x_cur=999999;
    double max_x_cur=-999999;
    for (size_t i_idx = 0; i_idx < m_inliers_vec[clust]->indices.size (); i_idx++) {
      if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x > max_x_cur  ){
        max_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
      }
      if ( clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x < min_x_cur  ){
        min_x_cur=clustered_clouds[clust]->points[m_inliers_vec[clust]->indices[i_idx]].x;
      }

    }
    std::cout << "cluster " << clust  << std::endl;
    std::cout << "min max " <<  min_x_cur << " " << max_x_cur << clust  << std::endl;

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

void Model::compute_rgb_colors(){

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

  for (vtkIdType i = 0; i < num_points; i++) {
    m_colors_active->InsertNextTuple(m_colors_original->GetTuple(i));
  }

  //m_wall->GetPointData()->SetScalars(colors_active);

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



  // int column_num = 1;
  // double max_dist = (*std::max_element(this->m_points_unwrapped.begin(), this->m_points_unwrapped.end(), column_comparer(column_num)))[column_num];
  // double min_dist = (*std::min_element(this->m_points_unwrapped.begin(), this->m_points_unwrapped.end(), column_comparer(column_num)))[column_num];
  //
  //
  // std::cout << "max, min dist is" << max_dist << " " << min_dist << std::endl;
  //
  // std::vector<double> depth(m_points_unwrapped);
  //
  // for (size_t i = 0; i < m_num_points; i++) {
  //   depth[i]=interpolate(this->m_points_unwrapped[i][1], min_dist, max_dist, 255.0, 0.0);
  // }
  //
  // m_colors_active->Reset();
  // m_colors_active = vtkSmartPointer<vtkUnsignedCharArray>::New();
  // m_colors_active->SetNumberOfComponents(3);
  // m_colors_active->SetName("depth");
  //
  //
  // std::cout << "starting to create colors" << std::endl;
  // for (size_t i = 0; i < m_num_points; i++) {
  //   //colors->InsertNextTuple3(angles[i]*255.0,0,0);
  //   m_colors_active->InsertNextTuple3(depth[i],depth[i],depth[i]);
  //   //m_points_unwrapped.InsertNextPoint( angles[i] *circumference,distances_from_radius[i],point[2])
  // }
  // std::cout << "finishing to create colors" << std::endl;


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

  //Detect cells that have now points inside
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
