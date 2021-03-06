#include "OBJReader2.h"
#include <iostream>

OBJReader2::OBJReader2():
m_polyData(vtkSmartPointer<vtkPolyData>::New()),
m_full_texture_name("full_texture.png"),
m_full_texture_original_name("full_texture_original.png"),
experimental_loading(true)
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
      //  std::cout << "OBJreader: reading texture file: " << m_path << tex_file_name  << std::endl;
       m_texture_file_names.push_back(m_path + tex_file_name);
     }

  }
  mtl_file.close();

}

void OBJReader2::read_textures(){
  m_textures.clear();
  int downsample=3;

  int max_rows=0, max_cols=0;

  for (size_t i = 0; i < m_texture_file_names.size(); i++) {
     cv::Mat img= cv::imread(m_texture_file_names[i]);
     cv::Size down_size(img.cols/downsample,img.rows/downsample);  //x,y
     cv::resize(img,img,down_size); //downsample each individual texture
     m_textures.push_back(img );
     if (img.rows > max_rows){
       max_rows=m_textures[i].rows;
     }
     if (img.cols > max_cols){
       max_cols=m_textures[i].cols;
     }
  }

  //Resize the individual textures to be the same size (biggest)
  cv::Size size(max_cols,max_rows);  //x,y
  for (size_t i = 0; i < m_textures.size(); i++) {
    cv::resize(m_textures[i],m_textures[i],size);//resize image
  }
  m_indiv_texture_size=max_rows;

  m_polys.resize(m_texture_file_names.size());  //polys will be clasified in vectors as many as we have materials

  std::cout << "finished reading textures" << std::endl;
  size_t sizeInBytes = m_textures[0].step[0] * m_textures[0].rows;
  std::cout << "size in bytes of first texture: " <<  sizeInBytes << std::endl;
  std::cout << "size in mb of first texture: " <<  sizeInBytes*1e-6 << std::endl;
  // std::cin.get();

  // std::cout << "objreader::read_textures" << std::endl;
  //
  // m_textures.clear();
  // int downsample=2;
  //
  // int max_rows=0, max_cols=0;
  //
  // for (size_t i = 0; i < m_texture_file_names.size(); i++) {
  //    cv::Mat img= cv::imread(m_texture_file_names[i]);
  //    if (img.rows > max_rows){
  //      max_rows=img.rows;
  //    }
  //    if (img.cols > max_cols){
  //      max_cols=img.cols;
  //    }
  // }
  //
  // m_polys.resize(m_texture_file_names.size());
  //
  // m_indiv_texture_size=max_rows/downsample;
}


void OBJReader2::create_full_texture(){
  // std::cout << "OBJ_READER::create_full_texture" << std::endl;
  //
  //  //Calculate how big should the full_texture be
  //  m_multiplier=ceil(sqrt(m_textures.size()));
  //
  //
  //  //make a texture big enough to get all of the small ones
  //  int t_rows=m_textures[0].rows;
  //  int t_cols=m_textures[0].cols;
  //  //32788
  //  std::cout << "full texture will be: " << t_rows*m_multiplier << " x " <<  t_cols*m_multiplier << std::endl;
  //  std::cout << "type of full texture is " <<  type2str(m_textures[0].type()) << std::endl;
  //  m_full_texture = cv::Mat::zeros(t_rows*m_multiplier, t_cols*m_multiplier, m_textures[0].type()) ;
  //
  //  //Copy the textures in a row by row manner.
  //   int x_idx=0, y_idx=-t_rows;
  //   for (size_t i = 0; i < m_textures.size(); i++) {
  //     if ((i)%m_multiplier==0){
  //       x_idx=0;
  //       y_idx+=t_rows;
  //     }else{
  //       x_idx+=t_cols;
  //     }
  //
  //     //According to the start of the img we need to get the offset taking into acount that opencv has origin at upper corner and opengl at lower corner
  //     double offset_x=(double)x_idx/m_full_texture.cols;
  //     double offset_y= (double)(m_full_texture.rows - (y_idx+t_rows))/m_full_texture.rows;
  //
  //     std::vector<double> offset_vec={offset_x,offset_y};
  //     m_tcoord_offsets.push_back(offset_vec);
  //
  //     m_textures[i].copyTo(m_full_texture(cv::Rect(x_idx, y_idx, t_cols, t_rows)));
  //   }
  //
  //   //Increase the exposure
  //   // m_full_texture = m_full_texture + cv::Scalar(75, 75, 75); //increase the brightness by 75 units
  //
  //   cv::imwrite( m_path+ m_full_texture_original_name, m_full_texture );
  //
  //   std::cout << "finished creating full texture" << std::endl;
  //   // std::cin.get();
  //
  //   // m_textures.clear();
  //
  //   fix_exposure();
  //
  //   // std::cout << "writing to file-----------" << m_path <<  "full_texture.png" << std::endl;
  //   cv::imwrite( m_path+ m_full_texture_name, m_full_texture );
  //
  //   m_full_texture.release();
  //   for (size_t i = 0; i < m_textures.size(); i++) {
  //     m_textures[i].release();
  //   }
  //   // m_textures.clear();
  //
  //   std::cout << "finished creating fixing exposure" << std::endl;
  //   // std::cin.get();




  // std::cout << "OBJ_READER::create_full_texture" << std::endl;
  //
  //  //Calculate how big should the full_texture be
  //  m_multiplier=ceil(sqrt(m_texture_file_names.size()));
  //
  //  std::cout << "objreader::create_full_texture: individual tex size is " << m_indiv_texture_size << std::endl;
  //
  //  //make a texture big enough to get all of the small ones
  //  int t_rows=m_indiv_texture_size;
  //  int t_cols=m_indiv_texture_size;
  //  //32788
  //  std::cout << "full texture will be: " << t_rows*m_multiplier << " x " <<  t_cols*m_multiplier << std::endl;
  // //  std::cout << "type of full texture is " <<  type2str(m_textures[0].type()) << std::endl;
  //  m_full_texture = cv::Mat::zeros(t_rows*m_multiplier, t_cols*m_multiplier, CV_8UC3) ;
  //
  //  //Copy the textures in a row by row manner.
  //   int x_idx=0, y_idx=-t_rows;
  //   for (size_t i = 0; i < m_texture_file_names.size(); i++) {
  //     if ((i)%m_multiplier==0){
  //       x_idx=0;
  //       y_idx+=t_rows;
  //     }else{
  //       x_idx+=t_cols;
  //     }
  //
  //     //According to the start of the img we need to get the offset taking into acount that opencv has origin at upper corner and opengl at lower corner
  //     double offset_x=(double)x_idx/m_full_texture.cols;
  //     double offset_y= (double)(m_full_texture.rows - (y_idx+t_rows))/m_full_texture.rows;
  //
  //     std::vector<double> offset_vec={offset_x,offset_y};
  //     m_tcoord_offsets.push_back(offset_vec);
  //
  //     cv::Mat img= cv::imread(m_texture_file_names[i]);
  //     cv::Size size(m_indiv_texture_size,m_indiv_texture_size);
  //     cv::resize(img,img,size);//resize image
  //
  //     img.copyTo(m_full_texture(cv::Rect(x_idx, y_idx, t_cols, t_rows)));
  //   }
  //
  //   //Increase the exposure
  //   // m_full_texture = m_full_texture + cv::Scalar(75, 75, 75); //increase the brightness by 75 units
  //
  //   cv::imwrite( m_path+ m_full_texture_original_name, m_full_texture );
  //
  //   std::cout << "finished creating full texture" << std::endl;
  //   // std::cin.get();
  //
  //   // m_textures.clear();
  //
  //   fix_exposure();
  //
  //   // std::cout << "writing to file-----------" << m_path <<  "full_texture.png" << std::endl;
  //   cv::imwrite( m_path+ m_full_texture_name, m_full_texture );
  //
  //   m_full_texture.release();
  //   // for (size_t i = 0; i < m_textures.size(); i++) {
  //   //   m_textures[i].release();
  //   // }
  //   // m_textures.clear();
  //
  //   std::cout << "finished creating fixing exposure" << std::endl;
  //   // std::cin.get();







    //Second way of creating the full textur by appending each individual texture in a column by column fashion
    std::cout << "OBJ_READER::create_full_texture" << std::endl;

   //Calculate how big should the full_texture be
   m_multiplier=m_texture_file_names.size();

   std::cout << "objreader::create_full_texture: individual tex size is " << m_indiv_texture_size << std::endl;

   //make a texture big enough to get all of the small ones
   int t_rows=m_indiv_texture_size;
   int t_cols=m_indiv_texture_size;
   //32788
   std::cout << "full texture will be: " << t_rows << " x " <<  t_cols*m_multiplier << std::endl;
  //  std::cout << "type of full texture is " <<  type2str(m_textures[0].type()) << std::endl;
   m_full_texture = cv::Mat::zeros( t_rows,  t_cols*m_multiplier, CV_8UC3) ;

   //Copy the textures in a column by column manner.  | 1 | 2 | 3 |
    int x_idx=0;
    for (size_t i = 0; i < m_texture_file_names.size(); i++) {

      //According to the start of the img we need to get the offset taking into acount that opencv has origin at upper corner and opengl at lower corner
      double offset_x=(double)x_idx/m_full_texture.cols;
      // double offset_y= (double)(m_full_texture.rows - (0+t_rows))/m_full_texture.rows;
      double offset_y = 0.0;

      std::vector<double> offset_vec={offset_x,offset_y};
      m_tcoord_offsets.push_back(offset_vec);

      m_textures[i].copyTo(m_full_texture(cv::Rect(x_idx, 0, t_cols, t_rows)));
      x_idx+=t_cols;
    }

    //Increase the exposure
    // m_full_texture = m_full_texture + cv::Scalar(75, 75, 75); //increase the brightness by 75 units

    cv::imwrite( m_path+ m_full_texture_original_name, m_full_texture );

    std::cout << "finished creating full texture" << std::endl;
    // std::cin.get();

    // m_textures.clear();

    fix_exposure();

    // std::cout << "writing to file-----------" << m_path <<  "full_texture.png" << std::endl;
    cv::imwrite( m_path+ m_full_texture_name, m_full_texture );

    m_full_texture.release();
    // for (size_t i = 0; i < m_textures.size(); i++) {
    //   m_textures[i].release();
    // }
    // m_textures.clear();

    std::cout << "finished creating fixing exposure" << std::endl;
    // std::cin.get();







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
  std::cout << "OBJReader2:: transform_tcoords" << std::endl;
  std::vector <int> tcoords_checked(m_tcoords.size(),0);

  //Loop through every material, though every face and change the tcoords taking into account the offset
  for (size_t mat_idx = 0; mat_idx < m_texture_file_names.size(); mat_idx++) {
    for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
      for (size_t point_idx = 0; point_idx < 3; point_idx++) {
        int tcoord_idx=m_polys[mat_idx][poly_idx][point_idx][1];


        if (!tcoords_checked[tcoord_idx]){
          m_tcoords[tcoord_idx][0]=m_tcoords[tcoord_idx][0]/m_multiplier;  //Squish the coordinates in x axis
          for (size_t t = 0; t < m_tcoords[0].size(); t++) {
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

  // int gl_idx=0;  //global index that just indicates point in the mesh
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
  //       vtk_points->InsertNextPoint(m_points[v_idx][0],m_points[v_idx][1],m_points[v_idx][2]);
  //
  //
  //       float tuple_n[3];
  //       tuple_n[0]=m_normals[vn_idx][0];
  //       tuple_n[1]=m_normals[vn_idx][1];
  //       tuple_n[2]=m_normals[vn_idx][2];
  //       // vtk_normals->InsertNextTuple(m_normals[i].data());
  //       vtk_normals->InsertNextTuple(tuple_n);
  //
  //
  //       float tuple_t[3];
  //       tuple_t[0]=m_tcoords[vt_idx][0];
  //       tuple_t[1]=m_tcoords[vt_idx][1];
  //       tuple_t[2]=m_tcoords[vt_idx][2];
  //       // vtk_tcoords->InsertNextTuple(m_tcoords[i].data());
  //       vtk_tcoords->InsertNextTuple(tuple_t);
  //
  //
  //       vtk_polys->InsertCellPoint(gl_idx);   //gl_idx will point to the previous point in the vtk_points
  //       gl_idx++;
  //     }
  //
  //
  //
  //   }
  //
  // }







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



  // //Attempt 3. Attempt 2 already workd but it can be made more readable

  //Calculate the maximum of v, vt and vn. If one is bigger than the others it indicates that the points might share that parameter in different faces. Eg: If we have more t_coords than points that means that one points will have different texture coordinates for different faces.
  //
  // int max_param;
  // int max_size=0;
  // if (m_points.size() >= max_size){
  //   max_size=m_points.size();
  //   max_param=0;
  // }
  // if (m_tcoords.size() >= max_size){
  //   max_size=m_tcoords.size();
  //   max_param=1;
  // }
  // if (m_normals.size() >= max_size){
  //   max_size=m_normals.size();
  //   max_param=2;
  // }
  //
  // std::cout << "obj_reader: maximum parameter is " << max_param << std::endl;
  // std::cout << "obj_reader: 0= points, 1= tcoords, 2=normals" << std::endl;
  //
  // std::cout << "v size " << m_points.size() <<  std::endl;
  // std::cout << "vt size " << m_tcoords.size() <<  std::endl;
  // std::cout << "vn size " << m_normals.size() <<  std::endl;
  // std::cout << "max size is " << max_size << std::endl;
  //
  // std::vector <int> point_is_added (max_size, 0);  //Checking on tcoord because the is the biggest size vector.
  //
  // int face=0;
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
  //       face++;
  //
  //       // std::cout << "adding cell: " << m_polys[mat_idx][poly_idx][point_idx][max_param] << std::endl;
  //       // if (face ==6)
  //       //   return;
  //
  //       vtk_polys->InsertCellPoint( m_polys[mat_idx][poly_idx][point_idx][max_param]  );   //normally it is vt_idx which will be the point in the mesh, Can be repeted and duplicated
  //     }
  //
  //   }
  //
  // }

  // std::cout << "OBJREADER: added points nr" << counter  << std::endl;




  // // //Attempt 4
  // //we cannot rely on calculting the max of the points or the normals because there are some points that are not references by any face.
  //
  // //Actual sizes of the points is done by looping through all the faces and storing the index for each paramenter
  // //the in thore 3 arrays of indices, calculate the numbers of 1s for each one of them. That is the number of points, t coords etc that is actually being used.
  //
  // // std::cout << "num_faces: " << num_faces_total << std::endl;
  //
  // //Get the paramenter that has the most points/values. Even though some of them might not be referenced in any face
  // int max=0;
  // if (m_points.size() > max){
  //   max=m_points.size();
  // }
  // if (m_tcoords.size() > max){
  //   max=m_tcoords.size();
  // }
  // if (m_normals.size() > max){
  //   max=m_normals.size();
  // }
  //
  // matrix_type_i indices(3);
  // indices[0].resize(max); //Make it big, just in case
  // indices[1].resize(max);
  // indices[2].resize(max);
  //
  //
  // for (size_t mat_idx = 0; mat_idx < m_textures.size(); mat_idx++) {
  //   for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
  //     for (size_t point_idx = 0; point_idx < 3; point_idx++) {
  //
  //       int v_idx= m_polys[mat_idx][poly_idx][point_idx][0];
  //       int vt_idx= m_polys[mat_idx][poly_idx][point_idx][1];
  //       int vn_idx= m_polys[mat_idx][poly_idx][point_idx][2];
  //
  //       indices[0][v_idx]=1;
  //       indices[1][vt_idx]=1;
  //       indices[2][vn_idx]=1;
  //     }
  //
  //   }
  //
  // }
  //
  // int num_points=std::count(indices[0].begin(), indices[0].end(), 1);
  // int num_tcoords=std::count(indices[1].begin(), indices[1].end(), 1);
  // int num_normals=std::count(indices[2].begin(), indices[2].end(), 1);
  //
  // // std::cout << "actual references to params are: " << num_points << " " << num_tcoords << " " << num_normals << std::endl;
  //
  //
  // int max_param;
  // int max_size=0;
  // if (num_points > max_size){
  //   max_size=num_points;
  //   max_param=0;
  // }
  // if (num_tcoords > max_size){
  //   max_size=num_tcoords;
  //   max_param=1;
  // }
  // if (num_normals > max_size){
  //   max_size=num_tcoords;
  //   max_param=2;
  // }
  //
  //
  // std::vector <int> point_is_added (max_size, 0);
  // std::vector <int> point_index(max_size, 0);
  //
  // int index=0;
  // for (size_t mat_idx = 0; mat_idx < m_textures.size(); mat_idx++) {
  //   for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
  //     vtk_polys->InsertNextCell(3);
  //     for (size_t point_idx = 0; point_idx < 3; point_idx++) {
  //
  //       int v_idx= m_polys[mat_idx][poly_idx][point_idx][0];
  //       int vt_idx= m_polys[mat_idx][poly_idx][point_idx][1];
  //       int vn_idx= m_polys[mat_idx][poly_idx][point_idx][2];
  //
  //       if (!point_is_added[  m_polys[mat_idx][poly_idx][point_idx][max_param]   ]){
  //
  //
  //         vtk_points->InsertNextPoint(m_points[v_idx].data());
  //         vtk_tcoords->InsertNextTuple(m_tcoords[vt_idx].data());
  //         vtk_normals->InsertNextTuple(m_normals[vn_idx].data());
  //
  //         point_is_added[m_polys[mat_idx][poly_idx][point_idx][max_param]]=1;
  //         point_index[m_polys[mat_idx][poly_idx][point_idx][max_param]]=index;
  //         vtk_polys->InsertCellPoint( index  );  //Keep increasing the index of the point when that points want added
  //         index ++;
  //
  //       }else{
  //         int stored_index=point_index[m_polys[mat_idx][poly_idx][point_idx][max_param]];
  //         vtk_polys->InsertCellPoint( stored_index  );  //If the point was already added then just point to it
  //       }
  //
  //     }
  //
  //   }
  //
  // }






  //Condition on both

  if (experimental_loading){
    int max=0;
    if (m_points.size() > max){
      max=m_points.size();
    }
    if (m_tcoords.size() > max){
      max=m_tcoords.size();
    }
    if (m_normals.size() > max){
      max=m_normals.size();
    }

    matrix_type_i indices(3);
    indices[0].resize(max); //Make it big, just in case
    indices[1].resize(max);
    indices[2].resize(max);


    for (size_t mat_idx = 0; mat_idx < m_texture_file_names.size(); mat_idx++) {
      for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
        for (size_t point_idx = 0; point_idx < 3; point_idx++) {

          int v_idx= m_polys[mat_idx][poly_idx][point_idx][0];
          int vt_idx= m_polys[mat_idx][poly_idx][point_idx][1];
          int vn_idx= m_polys[mat_idx][poly_idx][point_idx][2];

          indices[0][v_idx]=1;
          indices[1][vt_idx]=1;
          indices[2][vn_idx]=1;
        }

      }

    }

    int num_points=std::count(indices[0].begin(), indices[0].end(), 1);
    int num_tcoords=std::count(indices[1].begin(), indices[1].end(), 1);
    int num_normals=std::count(indices[2].begin(), indices[2].end(), 1);

    // std::cout << "actual references to params are: " << num_points << " " << num_tcoords << " " << num_normals << std::endl;


    int max_param;
    int max_size=0;
    if (num_points > max_size){
      max_size=num_points;
      max_param=0;
    }
    if (num_tcoords > max_size){
      max_size=num_tcoords;
      max_param=1;
    }
    if (num_normals > max_size){
      max_size=num_tcoords;
      max_param=2;
    }


    std::vector <int> point_is_added (max_size, 0);
    std::vector <int> point_index(max_size, 0);

    int index=0;
    for (size_t mat_idx = 0; mat_idx < m_texture_file_names.size(); mat_idx++) {
      for (size_t poly_idx = 0; poly_idx < m_polys[mat_idx].size(); poly_idx++) {
        vtk_polys->InsertNextCell(3);
        for (size_t point_idx = 0; point_idx < 3; point_idx++) {

          int v_idx= m_polys[mat_idx][poly_idx][point_idx][0];
          int vt_idx= m_polys[mat_idx][poly_idx][point_idx][1];
          int vn_idx= m_polys[mat_idx][poly_idx][point_idx][2];

          if (!point_is_added[  m_polys[mat_idx][poly_idx][point_idx][max_param]   ]){


            vtk_points->InsertNextPoint(m_points[v_idx].data());
            vtk_tcoords->InsertNextTuple(m_tcoords[vt_idx].data());
            vtk_normals->InsertNextTuple(m_normals[vn_idx].data());

            point_is_added[m_polys[mat_idx][poly_idx][point_idx][max_param]]=1;
            point_index[m_polys[mat_idx][poly_idx][point_idx][max_param]]=index;
            vtk_polys->InsertCellPoint( index  );  //Keep increasing the index of the point when that points want added
            index ++;

          }else{
            int stored_index=point_index[m_polys[mat_idx][poly_idx][point_idx][max_param]];
            vtk_polys->InsertCellPoint( stored_index  );  //If the point was already added then just point to it
          }

        }

      }

    }
  }else{



    int gl_idx=0;  //global index that just indicates point in the mesh
    for (size_t mat_idx = 0; mat_idx < m_texture_file_names.size(); mat_idx++) {
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

  }






  std::cout << "after" << std::endl;
  std::cout << "obj reader:: points total that will be written: " << vtk_points->GetNumberOfPoints()<< std::endl;
  std::cout << "obj reader:: tcoords total that will be written: " << vtk_tcoords->GetNumberOfTuples()<< std::endl;
  std::cout << "obj reader:: normals total that will be written: " << vtk_normals->GetNumberOfTuples()<< std::endl;



  // vtk_polys->UpdateCellCount(3);

  m_polyData->SetPoints(vtk_points);
  m_polyData->GetPointData()->SetNormals(vtk_normals);
  m_polyData->GetPointData()->SetTCoords(vtk_tcoords);
  m_polyData->SetPolys(vtk_polys);

  m_polyData->Squeeze();


}

void OBJReader2::fix_exposure(){


    // if(m_full_texture.channels() >= 3)
    // {
    //
    //     cv::Mat ycrcb;
    //     cv::cvtColor(m_full_texture,ycrcb,CV_BGR2YCrCb);
    //
    //     std::vector<cv::Mat> channels;
    //     cv::split(ycrcb,channels);
    //
    //     cv::equalizeHist(channels[0], channels[0]);
    //
    //     cv::Mat result;
    //     cv::merge(channels,ycrcb);
    //     cv::cvtColor(ycrcb,result,CV_YCrCb2BGR);
    //
    //     result.copyTo(m_full_texture);
    // }


    //attempt 2

  //   cv::Mat lab_image;
  //   cv::cvtColor(m_full_texture, lab_image, CV_BGR2Lab);
   //
  //   // Extract the L channel
  //   std::vector<cv::Mat> lab_planes(3);
  //   cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]
   //
  //   // apply the CLAHE algorithm to the L channel
  //   cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  //   clahe->setClipLimit(4);
  //   cv::Mat dst;
  //   // clahe->apply(lab_planes[0], dst);
   //
  //   cv::equalizeHist(lab_planes[0], lab_planes[0]);
   //
  //   // Merge the the color planes back into an Lab image
  //   // dst.copyTo(lab_planes[0]);
  //   cv::merge(lab_planes, lab_image);
   //
  //  // convert back to RGB
  //  cv::Mat image_clahe;
  //  cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);
   //
  //  image_clahe.copyTo(m_full_texture);


  //Attempt 3 hardcode
  //  m_full_texture = m_full_texture + cv::Scalar(35, 35, 35);
  //  int inc=10;
   //
  //  cv::Mat hsv;
  //  cvtColor(m_full_texture, hsv, CV_BGR2HSV);
   //
  //  for_each_pixel(hsv, [&inc](uchar * const pixel, int /*channels*/) {
  //      if (pixel[1] <= 255-inc)
  //          pixel[1] += inc;
  //      else
  //          pixel[1] = 255;
  //  });
   //
  //  cvtColor(hsv, m_full_texture, CV_HSV2BGR);




  //attempt 4 gimp strecth hsv
   if(m_full_texture.channels() >= 3)
   {

       cv::Mat hsv;
       cv::cvtColor(m_full_texture,hsv,CV_BGR2HSV);

       std::vector<cv::Mat> channels;
       cv::split(hsv,channels);

       //  cv::normalize(channels[0], channels[0], 0, 255, cv::NORM_MINMAX);
       cv::normalize(channels[1], channels[1], 0, 255, cv::NORM_MINMAX);
       cv::normalize(channels[2], channels[2], 0, 255, cv::NORM_MINMAX);


       cv::Mat result;
       cv::merge(channels,hsv);
       cv::cvtColor(hsv,result,CV_HSV2BGR);

       result.copyTo(m_full_texture);
   }

}


void OBJReader2::for_each_pixel(cv::Mat &image, std::function<void(uchar * const pixel, int channels)> fn)
{
    int rows     = image.rows;
    int cols     = image.cols;
    int channels = image.channels();

    if (image.isContinuous())
    {
        cols = cols * rows;
        rows = 1;
    }

    for (int j=0; j<rows; ++j)
    {
        auto pixel = image.ptr(j);
        for (int i=0; i<cols; ++i, pixel += channels)
            fn(pixel, channels);
    }
}


void OBJReader2::fix_orientation(){
  //some dataset are not centeres as we expected so rotate the mesh on the x axis

  vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
  trans->RotateX(90);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

  #if VTK_MAJOR_VERSION <= 5
      transformFilter->SetInputConnection(m_polyData->GetProducerPort());
  #else
      transformFilter->SetInputData(m_polyData);
  #endif


  transformFilter->SetTransform(trans);
  transformFilter->Update();
  m_polyData=transformFilter->GetOutput();

}

void OBJReader2::auto_fix_orientation(){
  std::cout << "auto fixing orientation of mesh" << std::endl;

  //look at the normals coordinates. The coorindate which mean is the lowest is probably the one that we should rotate by
}

void OBJReader2::decimate_mesh(){
  std::cout << "decimating mesh" << std::endl;


  vtkSmartPointer<vtkDataArray> tcoords = m_polyData->GetPointData()->GetTCoords();
  if(tcoords){
    std::cout << "decimate before:the polydata has tcoords" << std::endl;
  }else{
    std::cout << "decimate before:the polydata does not have tcoords" << std::endl;
  }


  vtkSmartPointer<vtkQuadricDecimation> decimate = vtkSmartPointer<vtkQuadricDecimation>::New();
  #if VTK_MAJOR_VERSION <= 5
   decimate->SetInputConnection(m_polyData->GetProducerPort());
  #else
   decimate->SetInputData(m_polyData);
  #endif

  decimate->TCoordsAttributeOn();
  decimate->Update();

  m_polyData=decimate->GetOutput();

  m_polyData->GetPointData()->SetTCoords(tcoords);

  vtkSmartPointer<vtkDataArray> tcoords_after = m_polyData->GetPointData()->GetTCoords();
  if(tcoords_after){
    std::cout << "decimate after:the polydata has tcoords" << std::endl;
  }else{
    std::cout << "decimate after:the polydata does not have tcoords" << std::endl;
  }

  decimate->Print(std::cout);


}


vtkSmartPointer<vtkPolyData> OBJReader2::GetOutput(){
  return m_polyData;
}

std::string OBJReader2::GetTexturePath(){
  std::string ret;
  ret=m_path + m_full_texture_name;
  return ret;
}

std::string OBJReader2::GetTextureOriginalPath(){
  std::string ret;
  ret=m_path + m_full_texture_original_name;
  return ret;

}
