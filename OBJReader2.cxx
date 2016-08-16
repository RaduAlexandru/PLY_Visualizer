#include "OBJReader2.h"
#include <iostream>

OBJReader2::OBJReader2():
m_polyData(vtkSmartPointer<vtkPolyData>::New())
{

}


void OBJReader2::SetFileName(std::string file_name){
  m_obj_file_name=file_name;

  std::size_t found = file_name.find_last_of("/\\");
  m_path=file_name.substr(0,found+1);
}


void OBJReader2::Update(){


  read_mtl_file();
  read_textures();
  create_full_texture();
  read_obj();
  transform_tcoords();
  write_to_poly();






  std::cout << "after reeading we have points " << m_points.size() << std::endl;
  std::cout << "after reeading we have normals " << m_normals.size() << std::endl;
  std::cout << "after reeading we have tcoords" << m_tcoords.size() << std::endl;
  std::cout << "after reeading we have polys" << m_polys.size() << std::endl;



  //pcl
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //
  // cloud->width    = sqrt(128468 +10);
  // cloud->height   = sqrt(128468 +10);
  // cloud->is_dense = false;
  // cloud->points.resize (cloud->width * cloud->height);
  //
  // for (size_t i = 0; i < m_tcoords.size (); ++i){
  //    cloud->points[i].x = m_tcoords[i][0];
  //    cloud->points[i].y =m_tcoords[i][1];
  //    cloud->points[i].z=0.0;
  // }
  //
  // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  // viewer.showCloud (cloud);
  // while (!viewer.wasStopped ())
  // {
  // }


  // for (size_t i = 0; i < 10; i++) {
  // std::cout << "points: " << m_tcoords[i][0] << " " << m_tcoords[i][1] << m_tcoords[i][2]  << std::endl;
  // }


  // for (size_t i = 0; i < m_tcoord_offsets.size(); i++) {
  //   std::cout << "t coord offser is " << i << " : " << m_tcoord_offsets[i][0] << " " << m_tcoord_offsets[i][1] << std::endl;
  // }
  //

  // for (size_t i = 0; i < 2; i++) {
  //   std::cout << "polys are " << std::endl;
  //
  //   for (size_t j = 0; j < 3; j++) {
  //     for (size_t k= 0; k < 3; k++) {
  //       std::cout << m_polys[0][i][j][k]   << std::endl;
  //     }
  //
  //   }
  //
  // }







  // cv::Size size(200,200);
  // cv::resize(m_full_texture, m_full_texture,size);//resize image
  // cv::namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );
  // cv::imshow( "Gray image", m_full_texture );
  // cv::waitKey(0);







}



void OBJReader2::read_mtl_file(){

  //Find the mtl file and open it
  std::string mtl_file_name;
  mtl_file_name = m_obj_file_name;
  mtl_file_name.erase(mtl_file_name.size() - 4);
  mtl_file_name+=".mtl";
  // std::cout << "looking for mtl file with name:" << mtl_file_name << std::endl;


  std::ifstream mtl_file;
  mtl_file.open(mtl_file_name.data());
  if (!mtl_file.is_open()) {
    std::cout << "could not open the mtl file" << std::endl;
  }

  while (mtl_file.good()) {
     std::stringstream ss;
     std::string line, word, tex_file_name;

     getline(mtl_file, line);
     ss << line;
     ss >> word >> tex_file_name;

     if (word=="map_Kd"){
       m_texture_file_names.push_back(m_path + tex_file_name);
     }

  }
  mtl_file.close();

}

void OBJReader2::read_textures(){
  for (size_t i = 0; i < m_texture_file_names.size(); i++) {
     m_textures.push_back( cv::imread(m_texture_file_names[i]) );
  }
  m_polys.resize(m_texture_file_names.size());  //polys will be clasified in vectors as many as we have materials
}


void OBJReader2::create_full_texture(){

   //Calculate how big should the full_texture be
   m_multiplier=ceil(sqrt(m_textures.size()));


   //make a texture big enough to get all of the small ones
   int t_rows=m_textures[0].rows;
   int t_cols=m_textures[0].cols;
   m_full_texture = cv::Mat::zeros(t_rows*m_multiplier, t_cols*m_multiplier, m_textures[0].type()) ;

   //Copy the textures in a row by row manner.
    int x_idx=0, y_idx=-t_rows;
    for (size_t i = 0; i < m_textures.size(); i++) {
      if ((i)%m_multiplier==0){
        x_idx=0;
        y_idx+=t_rows;
      }else{
        x_idx+=t_cols;
      }

      //According to the start of the img we need to get the offset taking into acount that opencv has origin at upper corner and opengl at lower corner
      double offset_x=(double)x_idx/m_full_texture.cols;
      double offset_y= (double)(m_full_texture.rows - (y_idx+t_rows))/m_full_texture.rows;

      std::vector<double> offset_vec={offset_x,offset_y};
      m_tcoord_offsets.push_back(offset_vec);

      m_textures[i].copyTo(m_full_texture(cv::Rect(x_idx, y_idx, t_cols, t_rows)));
    }
    // std::cout << "writing to file-----------" << m_path <<  "full_texture.png" << std::endl;
    cv::imwrite( m_path+ "full_texture.png", m_full_texture );

}


void OBJReader2::read_obj(){
   std::ifstream obj_file;
   obj_file.open(m_obj_file_name.data());
   if (!obj_file.is_open()) {
       std::cout << "could open the obj file" << std::endl;
   }


   int mat_idx=-1;

   while (obj_file.good()) {
      std::stringstream ss;
      std::string line, word;
      std::string x, y, z;

      getline(obj_file, line);
      ss << line;
      ss >> word >> x >> y >> z;

      std::vector <double> test(2);


      if (word == "v") {
        m_points.push_back(std::vector<double>{atof(x.data()),atof(y.data()),atof(z.data())});
      }

      if (word== "vn"){
        m_normals.push_back(std::vector<double>{atof(x.data()),atof(y.data()),atof(z.data())});

      }

      if (word== "vt"){
        m_tcoords.push_back(std::vector<double>{atof(x.data()),atof(y.data())});

      }

      if (word=="usemtl"){
        std::cout << "changing material" << std::endl;
        mat_idx++;
      }

      if (word== "f"){
        std::vector<std::string> split;
        // std::cout << "x= " << x << std::endl;
        // std::cout << "y= " << y << std::endl;
        // std::cout << "z= " << z << std::endl;

        matrix_type_i face;


        row_type_i face_point;

        boost::split(split, x, boost::is_any_of("/"));
        face_point={atoi(split[0].data())-1, atoi(split[1].data())-1, atoi(split[2].data())-1 };
        face.push_back(face_point);

        boost::split(split, y, boost::is_any_of("/"));
        face_point={atoi(split[0].data())-1, atoi(split[1].data())-1, atoi(split[2].data())-1 };
        face.push_back(face_point);

        boost::split(split, z, boost::is_any_of("/"));
        face_point={atoi(split[0].data())-1, atoi(split[1].data())-1, atoi(split[2].data())-1 };
        face.push_back(face_point);

        m_polys[mat_idx].push_back(face);


      }

    }

    obj_file.close();

}


void OBJReader2::transform_tcoords(){
  std::vector <int> tcoords_checked(m_tcoords.size(),0);

  //Loop through every material, though every face and change the tcoords taking into account the offset
  for (size_t mat_idx = 0; mat_idx < m_textures.size(); mat_idx++) {
    for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
      for (size_t point_idx = 0; point_idx < 3; point_idx++) {
        int tcoord_idx=m_polys[mat_idx][poly_idx][point_idx][1];

        // std::cout << "t coord" << tcoord_idx << std::endl;

        if (!tcoords_checked[tcoord_idx]){
          for (size_t t = 0; t < m_tcoords[0].size(); t++) {
            m_tcoords[tcoord_idx][t]=m_tcoords[tcoord_idx][t]/m_multiplier;
            m_tcoords[tcoord_idx][t]+=m_tcoord_offsets[mat_idx][t];
          }
          tcoords_checked[tcoord_idx]=1;
        }


      }
    }
  }

}

void OBJReader2::write_to_poly(){
  vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();

  vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
  vtk_normals->SetNumberOfComponents(3);
  vtk_normals->SetName("Normals");

  vtkSmartPointer<vtkFloatArray> vtk_tcoords = vtkSmartPointer<vtkFloatArray>::New();
  vtk_tcoords->SetName("TCoords");
  vtk_tcoords->SetNumberOfComponents(2);

  vtkSmartPointer<vtkCellArray> vtk_polys = vtkSmartPointer<vtkCellArray>::New();

  // for (size_t i = 0; i < m_points.size(); i++) {
  //   vtk_points->InsertNextPoint(m_points[i][0],m_points[i][1],m_points[i][2]);
  // }
  //
  //
  // for (size_t i = 0; i < m_normals.size(); i++) {
  //   float tuple[3];
  //   tuple[0]=m_normals[i][0];
  //   tuple[1]=m_normals[i][1];
  //   tuple[2]=m_normals[i][2];
  //
  //   // vtk_normals->InsertNextTuple(m_normals[i].data());
  //   vtk_normals->InsertNextTuple(tuple);
  // }
  //
  // for (size_t i = 0; i < m_tcoords.size(); i++) {
  //   float tuple[3];
  //   tuple[0]=m_tcoords[i][0];
  //   tuple[1]=m_tcoords[i][1];
  //   tuple[2]=m_tcoords[i][2];
  //   // vtk_tcoords->InsertNextTuple(m_tcoords[i].data());
  //   vtk_tcoords->InsertNextTuple(tuple);
  // }
  //
  //
  //
  // // vtk_polys->InsertNextCell(0);
  // for (size_t mat_idx = 0; mat_idx < m_textures.size(); mat_idx++) {
  //   for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
  //     // vtk_polys->InsertNextCell(0);
  //     // std::vector <int> face_vertices;
  //     for (size_t point_idx = 0; point_idx < 3; point_idx++) {
  //       // face_vertices.push_back(m_polys[mat_idx][poly_idx][point_idx][0]);
  //       int test=m_polys[mat_idx][poly_idx][point_idx][0];
  //       std::cout << "test is " << test  << std::endl;
  //       vtk_polys->InsertCellPoint(m_polys[mat_idx][poly_idx][point_idx][0]);
  //
  //
  //
  //
  //     }
  //
  //     // vtk_polys->InsertNextCell(3, face_vertices.data());
  //
  //     vtkIdType pts[3] = {m_polys[mat_idx][poly_idx][0][0], m_polys[mat_idx][poly_idx][1][0], m_polys[mat_idx][poly_idx][2][0]};
  //     vtk_polys->InsertNextCell(3,pts);
  //
  //   }
  // }
  //
  //
  // // vtk_polys->UpdateCellCount(m_points.size());
  // // vtk_tcoords->UpdateCellCount(m_tcoords.size());
  // // vtk_normals->UpdateCellCount(m_normals.size());
  //
  // m_polyData->SetPoints(vtk_points);
  // m_polyData->GetPointData()->SetNormals(vtk_normals);
  // m_polyData->GetPointData()->SetTCoords(vtk_tcoords);
  // m_polyData->SetPolys(vtk_polys);






  //get through the faces and grab the indeces for the v, vt and vn

  int gl_idx=0;  //global index that just indicates point in the mesh
  for (size_t mat_idx = 0; mat_idx < m_textures.size(); mat_idx++) {
    for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
      vtk_polys->InsertNextCell(3);
      for (size_t point_idx = 0; point_idx < 3; point_idx++) {

        int v_idx= m_polys[mat_idx][poly_idx][point_idx][0];
        int vt_idx= m_polys[mat_idx][poly_idx][point_idx][1];
        int vn_idx= m_polys[mat_idx][poly_idx][point_idx][2];



        vtk_points->InsertNextPoint(m_points[v_idx][0],m_points[v_idx][1],m_points[v_idx][2]);


        float tuple_n[3];
        tuple_n[0]=m_normals[vn_idx][0];
        tuple_n[1]=m_normals[vn_idx][1];
        tuple_n[2]=m_normals[vn_idx][2];
        // vtk_normals->InsertNextTuple(m_normals[i].data());
        vtk_normals->InsertNextTuple(tuple_n);


        float tuple_t[3];
        tuple_t[0]=m_tcoords[vt_idx][0];
        tuple_t[1]=m_tcoords[vt_idx][1];
        tuple_t[2]=m_tcoords[vt_idx][2];
        // vtk_tcoords->InsertNextTuple(m_tcoords[i].data());
        vtk_tcoords->InsertNextTuple(tuple_t);


        vtk_polys->InsertCellPoint(gl_idx);   //gl_idx will point to the previous point in the vtk_points
        gl_idx++;
      }



    }

  }

  std::cout << "finished filling the vtk_vectors" << std::endl;

  // vtk_polys->UpdateCellCount(3);

  m_polyData->SetPoints(vtk_points);
  m_polyData->GetPointData()->SetNormals(vtk_normals);
  m_polyData->GetPointData()->SetTCoords(vtk_tcoords);
  m_polyData->SetPolys(vtk_polys);

  m_polyData->Squeeze();

}

vtkSmartPointer<vtkPolyData> OBJReader2::GetOutput(){
  return m_polyData;
}
