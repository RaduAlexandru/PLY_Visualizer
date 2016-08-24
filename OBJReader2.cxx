#include "OBJReader2.h"
#include <iostream>

OBJReader2::OBJReader2():
m_polyData(vtkSmartPointer<vtkPolyData>::New()),
m_full_texture_name("full_texture.png")
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
    std::cout << "OBJ_READER: could not open the mtl file" << std::endl;
    return;
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
  std::cout << "OBJ_READER::read_textures" << std::endl;

  int max_rows=0, max_cols=0;

  for (size_t i = 0; i < m_texture_file_names.size(); i++) {
     m_textures.push_back( cv::imread(m_texture_file_names[i]) );
     if (m_textures[i].rows > max_rows){
       max_rows=m_textures[i].rows;
     }
     if (m_textures[i].cols > max_cols){
       max_cols=m_textures[i].cols;
     }
  }

  //Resize the individual textures to be the same size (biggest)
  cv::Size size(max_cols,max_rows);  //x,y
  for (size_t i = 0; i < m_textures.size(); i++) {
    cv::resize(m_textures[i],m_textures[i],size);//resize image
  }

  m_polys.resize(m_texture_file_names.size());  //polys will be clasified in vectors as many as we have materials
}


void OBJReader2::create_full_texture(){
  std::cout << "OBJ_READER::create_full_texture" << std::endl;

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
    cv::imwrite( m_path+ m_full_texture_name, m_full_texture );

}


void OBJReader2::read_obj(){
  std::cout << "OBJ_READER::read_obj" << std::endl;
   std::ifstream obj_file;
   obj_file.open(m_obj_file_name.data());
   if (!obj_file.is_open()) {
       std::cout << "OBJ_READER: could open the obj file" << std::endl;
       return;
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







  //Attempt 2 at writing into polydata in a more compact way
  // std::vector <int> vt_added (m_tcoords.size(), 0);  //Checking on tcoord because the is the biggest size vector.
  //
  // for (size_t mat_idx = 0; mat_idx < m_textures.size(); mat_idx++) {
  //   for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
  //     vtk_polys->InsertNextCell(3);
  //     for (size_t point_idx = 0; point_idx < 3; point_idx++) {
  //
  //       int v_idx= m_polys[mat_idx][poly_idx][point_idx][0];
  //       int vt_idx= m_polys[mat_idx][poly_idx][point_idx][1];
  //       int vn_idx= m_polys[mat_idx][poly_idx][point_idx][2];
  //
  //
  //
  //       if (!vt_added[vt_idx]){
  //         //vt
  //         float tuple_t[3];
  //         tuple_t[0]=m_tcoords[vt_idx][0];
  //         tuple_t[1]=m_tcoords[vt_idx][1];
  //         tuple_t[2]=m_tcoords[vt_idx][2];
  //         vtk_tcoords->InsertNextTuple(tuple_t);
  //         vt_added[vt_idx]=1;
  //
  //         //v
  //         vtk_points->InsertNextPoint(m_points[v_idx][0],m_points[v_idx][1],m_points[v_idx][2]);
  //
  //         //vn
  //         float tuple_n[3];
  //         tuple_n[0]=m_normals[vn_idx][0];
  //         tuple_n[1]=m_normals[vn_idx][1];
  //         tuple_n[2]=m_normals[vn_idx][2];
  //         // vtk_normals->InsertNextTuple(m_normals[i].data());
  //         vtk_normals->InsertNextTuple(tuple_n);
  //       }
  //
  //
  //       vtk_polys->InsertCellPoint(vt_idx);   //adding vt_idx which will be the point in the mesh, Can be repeted and duplicated
  //     }
  //
  //   }
  //
  // }



  //Attempt 3. Attempt 2 already workd but it can be made more readable

  //Calculate the maximum of v, vt and vn. If one is bigger than the others it indicates that the points might share that parameter in different faces. Eg: If we have more t_coords than points that means that one points will have different texture coordinates for different faces.
  //
  // int max_param;
  // int max_size=0;
  // if (m_points.size() > max_size){
  //   max_size=m_points.size();
  //   max_param=0;
  // }
  // if (m_tcoords.size() > max_size){
  //   max_size=m_tcoords.size();
  //   max_param=1;
  // }
  // if (m_normals.size() > max_size){
  //   max_size=m_normals.size();
  //   max_param=2;
  // }
  //
  // std::cout << "obj_reader: maximum parameter is " << max_param << std::endl;
  // std::cout << "obj_reader: 0= points, 1= tcoords, 2=normals" << std::endl;
  // std::cout << "v size " << m_points.size() <<  std::endl;
  // std::cout << "vt size " << m_tcoords.size() <<  std::endl;
  // std::cout << "vn size " << m_normals.size() <<  std::endl;
  //
  // std::vector <int> point_is_added (max_size, 0);  //Checking on tcoord because the is the biggest size vector.
  //
  //
  // int counter=0;
  // for (size_t mat_idx = 0; mat_idx < m_textures.size(); mat_idx++) {
  //   for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
  //     vtk_polys->InsertNextCell(3);
  //     for (size_t point_idx = 0; point_idx < 3; point_idx++) {
  //
  //       int v_idx= m_polys[mat_idx][poly_idx][point_idx][0];
  //       int vt_idx= m_polys[mat_idx][poly_idx][point_idx][1];
  //       int vn_idx= m_polys[mat_idx][poly_idx][point_idx][2];
  //
  //
  //
  //       if (!point_is_added[  m_polys[mat_idx][poly_idx][point_idx][max_param]   ]){
  //         // std::cout << "adding points " << counter << std::endl;
  //         vtk_points->InsertNextPoint(m_points[v_idx][0],m_points[v_idx][1],m_points[v_idx][2]);
  //         vtk_tcoords->InsertNextTuple(m_tcoords[vt_idx].data());
  //         vtk_normals->InsertNextTuple(m_normals[vn_idx].data());
  //
  //         point_is_added[m_polys[mat_idx][poly_idx][point_idx][max_param]]=1;
  //         counter++;
  //       }
  //
  //
  //       vtk_polys->InsertCellPoint( m_polys[mat_idx][poly_idx][point_idx][max_param]  );   //normally it is vt_idx which will be the point in the mesh, Can be repeted and duplicated
  //     }
  //
  //   }
  //
  // }
  //
  // std::cout << "OBJREADER: added points nr" << counter  << std::endl;



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

std::string OBJReader2::GetTexturePath(){
  std::string ret;
  ret=m_path + m_full_texture_name;
  return ret;
}
