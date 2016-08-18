#include "Visualizer.h"

// This is included here because it is forward declared in
// Visualizer.h
#include "ui_Visualizer.h"





template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

  return idx;
}




Visualizer::Visualizer():
  model(new Model()),
  renderer(vtkSmartPointer<vtkRenderer>::New()),
  interactor(vtkSmartPointer<InteractorPointPicker>::New())
{
  this->ui = new Ui_Visualizer;
  this->ui->setupUi(this);

  this->ui->colorComboBox->addItem("Plain");
  this->ui->colorComboBox->addItem("RGB");
  this->ui->colorComboBox->addItem("Depth");
  this->ui->colorComboBox->addItem("Curvature");


  // // //ui
  // // self.centralWidget = QtGui.QWidget(MainWindow)
  // // self.gridlayout = QtGui.QGridLayout(self.centralWidget)
  // // self.vtkWidget = QVTKRenderWindowInteractor(self.centralWidget)
  // // self.gridlayout.addWidget(self.vtkWidget, 0, 0, 100, 100)
  // // self.buttonLeft = QtGui.QPushButton("Left")
  // // self.gridlayout.addWidget(self.buttonLeft, 96,48,1,1)
  // // self.buttonRight = QtGui.QPushButton("Right")
  // // self.gridlayout.addWidget(self.buttonRight, 96,52,1,1)
  // // self.buttonUp= QtGui.QPushButton("Up")
  // // self.gridlayout.addWidget(self.buttonUp, 94,50,1,1)
  // // self.buttonDown = QtGui.QPushButton("Down")
  // // self.gridlayout.addWidget(self.buttonDown, 98,50,1,1)
  // // self.buttonFire = QtGui.QPushButton("Fire Torpedo")
  // // self.gridlayout.addWidget(self.buttonFire, 95,50,3,1)
  // // MainWindow.setCentralWidget(self.centralWidget)
  //
  //
  // QGridLayout *layout = new QGridLayout;
  //
  // // CAM111 = new QLabel("CAM 01");
  //
  // // for (int i = 1; i < 10; ++ i) {
  // //       QLabel * const label = new QLabel(QString("CAM %1").arg(i, 2, 10, QLatin1Char('0')));
  // //       label->setFixedSize(200, 50);
  // //       layout->addWidget(label, (i-1) / 3, (i-1) % 3);
  // //       cams << label;
  // //   }
  //
  // layout->addWidget(new QPushButton("Button Text"), 0, 0);
  // layout->addWidget(new QVTKRenderWindowInteractor(), 1, 1);
  //
  // QWidget * central = new QWidget();
  // setCentralWidget(central);
  // centralWidget()->setLayout(layout);
  //
  // setWindowTitle("Camera Window");
  // setFixedSize(1000, 800);




  // VTK Renderer
  renderer->GradientBackgroundOn();
  renderer->SetBackground(1.0,1.0,1.0);
  renderer->SetBackground2(0.1,0.1,0.1);



  // Changing interactor to a custom one to handle different mouse events
  //vtkSmartPointer<vtkPointPicker> pointPicker = vtkSmartPointer<vtkPointPicker>::New();
  vtkSmartPointer<vtkCellPicker> pointPicker = vtkSmartPointer<vtkCellPicker>::New();
  this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle( interactor );
  this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(pointPicker);
  interactor->selecting_defects=&(model->selecting_defects);



  renderer->GetActiveCamera()->SetParallelProjection(0);

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

  // Set up action signals and slots
  connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

}


void Visualizer::on_loadFileButton_clicked(){
  // QString fileName;
	// fileName = QFileDialog::getOpenFileName(this,
	// 	tr("Open File"), "./", tr("File (*.*)"));
  //
	// if (fileName.isEmpty()){
  //   return;
  // }
  //
  // std::cout << "filename: " << fileName.toStdString() << std::endl;
  //
  //
  //
  // if (boost::ends_with(fileName.toStdString(), ".ply")) {
  //   std::cout << "reading .ply file" << std::endl;
  // }else if (boost::ends_with(fileName.toStdString(), ".obj")){
  //   std::cout << "reading .obj file" << std::endl;
  // }else{
  //   std::cout << "NOT VALID FORMAT" << std::endl;
  //   return;
  // }

  //   vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  //   // vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
  //   reader->SetFileName ( fileName.toStdString().c_str() );
  //   reader->Update();
  //   //reader->GetOutput()->GetPointData()->SetNormals(NULL);
  //
  //
  //   //cut the cylinder
  //   vtkSmartPointer<vtkBox> box_cut = vtkSmartPointer<vtkBox>::New();
  // //  box_cut->SetBounds (0.0, 1000, -0.005, 0.005, -1000.0, 1000.0);
  //    box_cut->SetBounds (0.0, 1000, -0.007, 0.007, -1000.0, 1000.0);
  //   vtkSmartPointer<vtkClipPolyData> clipper= vtkSmartPointer<vtkClipPolyData>::New();
  //   clipper->SetInputConnection(reader->GetOutputPort());
  //   clipper->SetClipFunction(box_cut);
  //   clipper->InsideOutOff();
  //   clipper->Update();
  //   clipper->GenerateClippedOutputOn();
  //
  //   model->set_mesh(clipper->GetOutput());
  //
  //   ui->colorComboBox->setCurrentIndex(ui->colorComboBox->findText("RGB"));
  //
  //
  //   // //trying to detect the bug in the reader
  //   // std::string inputFilename= "/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/result/result.ply";
  //   // vtkSmartPointer<vtkPLYReader> reader =
  //   //   vtkSmartPointer<vtkPLYReader>::New();
  //   // reader->SetFileName ( inputFilename.c_str() );
  //   // reader->Update();
  //   // vtkSmartPointer<vtkPolyDataMapper> mapper =
  //   //   vtkSmartPointer<vtkPolyDataMapper>::New();
  //   // mapper->SetInputConnection(reader->GetOutputPort());
  //   // vtkSmartPointer<vtkActor> actor =
  //   //   vtkSmartPointer<vtkActor>::New();
  //   // actor->SetMapper(mapper);
  //   // renderer->AddActor(actor);
  //   // this->ui->qvtkWidget->GetRenderWindow()->Render();
  //
  //
  //   updateView();

  //-----NOW WE START DOING THE WEIRD STUFF----------------------------------------------------------------------










  //BRUTE FORCE reading
 // std::string inputFilename= "/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/research_textured_mesh/researchDenslyTexturedMesh.obj";
 // std::string texFilename= "/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/dense_ir_result/texturedMesh_material0000_map_Kd.png";
 //
 // // std::string inputFilename= "/media/alex/Nuevo_vol/Master/SHK/Data/Custom/custom_cyl_5_high_res.ply";
 //
 // // vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
 // vtkSmartPointer<vtkOBJReader> reader= vtkSmartPointer<vtkOBJReader>::New();
 //
 // reader->SetFileName ( inputFilename.data() );
 // reader->Update();
 //
 // // Visualize
 // vtkSmartPointer<vtkPolyDataMapper> mapper =
 //   vtkSmartPointer<vtkPolyDataMapper>::New();
 // mapper->SetInputConnection(reader->GetOutputPort());
 //
 // vtkSmartPointer<vtkActor> actor =
 //   vtkSmartPointer<vtkActor>::New();
 // actor->SetMapper(mapper);
 //
 //
 //
 //
 // // //texture
 // // vtkSmartPointer<vtkPNGReader> texReader =
 // // vtkSmartPointer<vtkPNGReader>::New();
 // // texReader->SetFileName(texFilename.c_str());
 // // texReader->Update();
 // //
 // // vtkSmartPointer<vtkTexture> colorTexture =
 // // vtkSmartPointer<vtkTexture>::New();
 // // colorTexture->SetInputConnection(texReader->GetOutputPort());
 // // colorTexture->InterpolateOn();
 // // actor->SetTexture(colorTexture);
 //
 //   renderer->RemoveAllViewProps();
 //   renderer->AddActor(actor);
 //   renderer->GetActors()->GetLastActor()->GetProperty()->BackfaceCullingOn();
 //   renderer->TwoSidedLightingOff	();
 //   renderer->ResetCamera();
 //   this->ui->qvtkWidget->GetRenderWindow()->Render();
 //
 //   model->set_mesh(reader->GetOutput());












  //OBJ IMPORTER
  // //  std::string inputFilename= "/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/dense_ir_result/texturedMesh.obj";
  // //  std::string texFilename= "/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/dense_ir_result/texturedMesh_material0000_map_Kd.png";
  //
  // //  std::string filenameOBJ="/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/dense_ir_result/texturedMesh.obj";
  // //  std::string filenameMTL="/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/dense_ir_result/texturedMesh.mtl";
  // //  std::string texture_path1="/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/dense_ir_result/";
  //
  //
  //  std::string filenameOBJ="/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/researchDenslyTexturedMesh.obj";
  //  std::string filenameMTL="/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/researchDenslyTexturedMesh.mtl";
  //  std::string texture_path1="/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/";
  //
  //
  //
  //  vtkSmartPointer<vtkOBJImporter> reader = vtkSmartPointer<vtkOBJImporter>::New();
  //  reader->SetFileName(filenameOBJ.data());
  //  reader->SetFileNameMTL(filenameMTL.data());
  //  reader->SetTexturePath(texture_path1.data());
  //  reader->SetRenderWindow(this->ui->qvtkWidget->GetRenderWindow());
  //  reader->Update();
  //
  //  int num= renderer->GetActors()->GetNumberOfItems();
  //  std::cout << "num of actors" << num << std::endl;
  //
  //  //Backface cull al the actors
  //  vtkSmartPointer<vtkActorCollection> actorCollection = renderer->GetActors();
  //  actorCollection->InitTraversal();
  // //  for(vtkIdType i = 0; i < actorCollection->GetNumberOfItems(); i++){
  // //    vtkActor* actor = actorCollection->GetNextActor();
  // //    actor->GetProperty()->BackfaceCullingOn();
  // //  }
  //
  //  //now we have 4 actors from which we need the polydata and the colors
  //  vtkActor* actor_part;
  //  actor_part= actorCollection->GetNextActor();
  //  actor_part= actorCollection->GetNextActor();
  //
  //  vtkSmartPointer<vtkPolyData> part1 = vtkPolyData::SafeDownCast(
  //     actor_part->GetMapper()->GetInputAsDataSet());
  //
  // //  vtkSmartPointer<vtkUnsignedCharArray> colors_actor_1=
  // //  actor_part->GetMapper()->MapScalars(1.0) ;
  // //
  // //  if(!colors_actor_1){
  // //   std::cout << "LACKING IN color " << std::endl;
  // // }else{
  // //   std::cout << "WE got color!!!! " << std::endl;
  // // }
  //
  //
  //  vtkSmartPointer<vtkTexture> colorTexture = vtkSmartPointer<vtkTexture>::New();
  //  colorTexture= actor_part->GetTexture();
  //
  // //  part1->GetPointData()->SetScalars(colors_actor_1);
  //
  //
  //
  // //Get texture coordinates
  // vtkSmartPointer<vtkFloatArray> tc = vtkSmartPointer<vtkFloatArray>::New();
  // tc=vtkFloatArray::SafeDownCast( part1->GetPointData()->GetTCoords( ) );
  // // tc->FastDelete();
  // if(!tc){
  //  std::cout << "LACKING IN texcoordinates " << std::endl;
  // }else{
  //  std::cout << "WE got texcoordinates!!!! " << std::endl;
  // }
  //
  // //Show those tex coorindates
  // std::cout << "number of texture coordiantes " << tc->GetSize() << std::endl;
  // std::cout << "number of vertices " << part1->GetNumberOfPoints() << std::endl;
  // //  grab the texture coords only of the vertices that are actually in the polydata. Otherwise you risk reading something unusual
  // for (vtkIdType i = 0; i < part1->GetNumberOfPoints(); i++) {
  //   double tex_coord[2];
  //   tc->GetTuple (i, tex_coord);
  //   // std::cout << "tex coord at index " << i << " is " << tex_coord[0] << " " << tex_coord[1] << std::endl;
  //   // std::cout << "tex coord is "  << std::endl;
  //
  //   // double* pixel = static_cast<double*>(colorTexture->GetInput()->GetScalarPointer(tex_coord[1],tex_coord[0],0));
  //   // // do something with v
  //   // double v= pixel[1];
  //   // std::cout << "v is: " << v << std::endl;
  // }
  //
  // std::cout << "finished reading the tex coordinates " << std::endl;
  //
  //
  //
  // //use the tex coords to get the raw colors at those points
  // // int* dims = colorTexture->GetInput()->GetDimensions();
  // // std::cout << "dimes are z: " <<  dims[2] << " y: " << dims[1] << " x: " << dims[0] << std::endl;
  // // for (int z = 0; z < dims[2]; z++)
  // //     {
  // //     for (int y = 0; y < dims[1]; y++)
  // //       {
  // //       for (int x = 0; x < dims[0]; x++)
  // //         {
  // //         double* pixel = static_cast<double*>(colorTexture->GetInput()->GetScalarPointer(x,y,z));
  // //         // do something with v
  // //         double v= pixel[0];
  // //
  // //         // if (v!=0){
  // //         //   std::cout << "v is " << v << std::endl;
  // //         // }
  // //
  // //         // std::cout << pixel[0] << " ";
  // //         }
  // //       // std::cout << std::endl;
  // //       }
  // //     // std::cout << std::endl;
  // //     }
  //
  //
  //
  // //Make an assembly
  // // vtkSmartPointer<vtkAssembly> assembly =
  // //   vtkSmartPointer<vtkAssembly>::New();
  // // assembly->AddPart(actor_part);
  // // assembly->AddPart(actorCollection->GetNextActor());
  // // assembly->AddPart(actorCollection->GetNextActor());
  // //
  //
  // //
  // // vtkSmartPointer<vtkPolyData> total = vtkPolyData::SafeDownCast(
  // //    assembly->GetMapper()->GetInputAsDataSet());
  //
  //
  //  //Show the newly made Ones
  //  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  //  mapper->SetInputData(part1);
  //
  //  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  //  actor->SetMapper(mapper);
  //  actor->SetTexture(colorTexture);
  //
  //
  // //  renderer->RemoveAllViewProps();
  // //  renderer->AddActor(actor);
  // //  renderer->AddActor(assembly);
  //
  //  renderer->ResetCamera();
  //  //renderer->SetTwoSidedLighting(0);
  //  this->ui->qvtkWidget->GetRenderWindow()->Render();






   //-------NEW VTK HELPER------------------------------------------------------
  //  //Read obj filename
  //  //With the filename, get mtl filename and read it
  //  //
  //
  //  //Get the textures
  //  cv::Mat tex_0 = cv::imread("/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/tex_0.png", CV_LOAD_IMAGE_GRAYSCALE);
  //  cv::Mat tex_1 = cv::imread("/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/tex_1.png", CV_LOAD_IMAGE_GRAYSCALE);
  //  cv::Mat tex_2 = cv::imread("/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/tex_2.png", CV_LOAD_IMAGE_GRAYSCALE);
  //
  //  std::vector <cv::Mat> textures;
  //  textures.push_back(tex_0);
  //  textures.push_back(tex_1);
  //  textures.push_back(tex_2);
  //
  //
  //  //Calculate how big should the full_texture need to be
  //  int multiplier;
  //  multiplier=ceil(sqrt(textures.size()));
  //  std::cout << "nr of images is" << textures.size() << std::endl;
  //  std::cout << "multiplier is" << multiplier << std::endl;
  //
  //
  //  //make a texture big to get all of the small ones
  //  int t_rows=textures[0].rows;
  //  int t_cols=textures[0].cols;
  //  cv::Mat full_texture = cv::Mat::zeros(t_rows*multiplier, t_cols*multiplier, textures[0].type()) ;
  //  std::cout << "full texture has rows and cols: " << full_texture.rows << " " << full_texture.cols << std::endl;
  //
  //  //Copy them in a row by row manner.
  //  int x_idx=0, y_idx=-t_rows;
  //  matrix_type texture_offsets;
  //  for (size_t i = 0; i < textures.size(); i++) {
  //    if ((i)%multiplier==0){
  //     //  std::cout << "texture " << i << " goes into next row " << std::endl;
  //      x_idx=0;
  //      y_idx+=t_rows;
  //    }else{
  //      x_idx+=t_cols;
  //    }
  //   //  std::cout << "texture " << i << "start at " << x_idx << " " << y_idx << std::endl;
  //
  //    //According to the start of the img we need to get the offset taking into acount that opencv has origin at upper corner and opengl at lower corner
  //    double offset_x=(double)x_idx/full_texture.cols;
  //    double offset_y= (double)(full_texture.rows - (y_idx+t_rows))/full_texture.rows;
  //    //  std::cout << "texture " << i  << " "<< offset_x << " " << offset_y << std::endl;
  //    std::vector<double> offset_vec={offset_x,offset_y};
  //    texture_offsets.push_back(offset_vec);
  //
  //   //  cv::flip(textures[i],textures[i],1);
  //
  //    textures[i].copyTo(full_texture(cv::Rect(x_idx, y_idx, t_cols, t_rows)));
  //  }
  // //  cv::flip(full_texture,full_texture,0);
  //  cv::imwrite( "./full_texture.png", full_texture );
  //
  //
  //
  //
  //  //Read the obj file and the t coords
  //
  //
  //  //read all the vt in one big array
  //  //create an array of checked to see if I changed all of them
  //
  //  //read the obj again and now look for face and which material they have.
  //  //modify each vt accoringly
  //
  //
  //  std::string file= "/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/researchDenslyTexturedMesh_new.obj";
  //  std::ifstream objFile;
  //    objFile.open(file.data());
  //    if (!objFile.is_open()) {
  //        std::cout << "could open the obj file" << std::endl;
  //   }
  //
  //
  //
  //  //Read all coords
  //  matrix_type coords;
  //  std::vector <double> coords_line(2);
  //  while (objFile.good()) {
  //    std::stringstream ss;
  //    std::string line;
  //    std::string word;
  //    float x, y;
  //
  //    getline(objFile, line);
  //    ss << line;
  //    ss >> word >> x >> y;
  //
  //
  //    if (word == "vt") {
  //      coords_line[0]=x;
  //      coords_line[1]=y;
  //      coords.push_back(coords_line);
  //    }
  //
  //  }
  //  std::cout << "read a total of vts: " << coords.size() << std::endl;
  //
  //
  //  //Read the faces
  //  objFile.clear();
  //  objFile.seekg(0);
  //  int mat_idx=-1;
  //  std::vector <int> coords_checked(coords.size(),0);
  //  std::vector <int> vertex_indices;
  //  while (objFile.good()) {
  //    std::stringstream ss;
  //    std::string line;
  //    std::string word;
  //
  //    std::vector<std::string> face_points(3);
  //
  //    getline(objFile, line);
  //    ss << line;
  //    ss >> word >> face_points[0] >> face_points[1] >> face_points[2];
  //
  //
  //    if (word=="usemtl"){
  //      std::cout << "changing material" << std::endl;
  //      mat_idx++;
  //    }
  //
  //    if (word == "f") {
  //     //  std::cout << "face has " << face_points[0] << " " <<  face_points[1] << " " << face_points[2] << std::endl;
  //
  //      //Iterate through those points get the vt stores in them
  //      for (size_t p_idx = 0; p_idx < face_points.size(); p_idx++) {
  //        std::vector<std::string> strs;
  //        boost::split(strs, face_points[p_idx], boost::is_any_of("/"));
  //       //  std::cout << "splitted " << face_points[p_idx] << " into " << strs[1] << std::endl;
  //        int coord_to_change=atoi(strs[1].data());
  //        coord_to_change=coord_to_change-1; // To account for the fact that obj faces start counting from 1
  //
  //        int vert_index=atoi(strs[0].data());
  //        vert_index=vert_index-1;
  //       //  vertex_indices[vert_index]=coord_to_change;
  //       vertex_indices.push_back(vert_index);
  //
  //
  //       //  std::cout << "changing coorindate " << vert_index  << " " << coord_to_change << std::endl;
  //
  //        //not hcange that coorindate only if it hasn't been already changed
  //        if (coords_checked[coord_to_change]==0){
  //          coords_checked[coord_to_change]=1;
  //
  //
  //
  //          //also need to divide all the coordinats by the multiplier so as the resize them.
  //          coords[coord_to_change][0]=coords[coord_to_change][0]/multiplier;
  //          coords[coord_to_change][1]=coords[coord_to_change][1]/multiplier;
  //
  //          coords[coord_to_change][0]+=texture_offsets[mat_idx][0];
  //          coords[coord_to_change][1]+=texture_offsets[mat_idx][1];
  //
  //        }
  //
  //      }
  //
  //    }
  //
  //  }
  //  objFile.close();
  //  std::cout << "finished with tex coordins" << std::endl;
  //
  //
  //
  // //  for (size_t i = coords.size()-15; i < coords.size(); i++) {
  // //    std::cout << "coords " << i <<  " : " << coords[i][0] << " " << coords[i][1]  << std::endl;
  //  //
  // //  }
  //
  //
  //  //Reoder them correctly taking into account that they should be in the order of the vertices
  //  matrix_type coords_ordered;
  //  for (size_t i = 0; i < vertex_indices.size(); i++) {
  //    int vert=vertex_indices[i];
  //     // coords_ordered[i]=coords[vert];
  //   //  coords_ordered[i].push_back(coords[vert][0]);
  //   //  coords_ordered[i].push_back(coords[vert][1]);
  //   coords_ordered.push_back(coords[vert]);
  //  }
  //
  //
  //
  //
  //  //Add the texture coordinates back to the original obj
  //  vtkSmartPointer<vtkOBJReader> reader= vtkSmartPointer<vtkOBJReader>::New();
  //  reader->SetFileName ( file.data() );
  //  reader->Update();
  //
  //  vtkSmartPointer<vtkFloatArray> textureCoordinates = vtkSmartPointer<vtkFloatArray>::New();
  //  textureCoordinates->SetNumberOfComponents(2);
  //  textureCoordinates->SetName("TextureCoordinates");
  //
  //  for (size_t c_idx = 0; c_idx < coords_ordered.size(); c_idx++) {
  //    float tuple[2] = {(float)coords_ordered[c_idx][0], (float)coords_ordered[c_idx][1]};
  //    textureCoordinates->InsertNextTuple(tuple);
  //  }
  //
  // //  reader->GetOutput()->GetPointData()->SetTCoords(textureCoordinates);
  //
  //
  //
  //  vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
  //  std::string full_texture_file="/media/alex/Data/Master/SHK/vtk_scripts/RenderWindowUISingleInheritance/build/full_texture.png";
  // //  std::string full_texture_file="/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/tex_0.png";
  //  pngReader->SetFileName (full_texture_file.data() );
  //  pngReader->Update();
  //
  //  vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
  //  texture->SetInputConnection(pngReader->GetOutputPort());
  //
  //
  //  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  //  mapper->SetInputData(reader->GetOutput());
  //
  //  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  //  actor->SetMapper(mapper);
  //  actor->SetTexture(texture);
  //
  //  renderer->RemoveAllViewProps();
  //  renderer->AddActor(actor);
  //
  //  renderer->ResetCamera();
  //  this->ui->qvtkWidget->GetRenderWindow()->Render();
  //





   //pcl
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   //
  //  cloud->width    = sqrt(128468 +10);
  //  cloud->height   = sqrt(128468 +10);
  //  cloud->is_dense = false;
  //  cloud->points.resize (cloud->width * cloud->height);
   //
  //  for (size_t i = 0; i < coords.size (); ++i){
  //     cloud->points[i].x = coords[i][0];
  //     cloud->points[i].y =coords[i][1];
  //     cloud->points[i].z=0.0;
  //  }
   //
  //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  //  viewer.showCloud (cloud);
  //  while (!viewer.wasStopped ())
  //  {
  //  }



  //write them to file
  // std::ofstream myfile;
  // myfile.open ("./example.txt");
  // for (size_t i = 0; i < coords.size (); ++i){
  //     myfile << "vt " << coords[i][0] << " " << coords[i][1] << "\n";
  //  }
  // myfile.close();
  //



  //  cv::Size size(200,200);
  //  cv::resize(full_texture,full_texture,size);//resize image
  //  cv::namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );
  //  cv::imshow( "Gray image", full_texture );
  // //  waitKey(0);



  //OWN OBJ IMPORTER-------------------------------------------------------------------------------------------------
  std::string file_name= "/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/researchDenslyTexturedMesh.obj";

  std::unique_ptr<OBJReader2> obj_reader(new OBJReader2());

  obj_reader->SetFileName(file_name);
  obj_reader->Update();


  vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
  pngReader->SetFileName (obj_reader->GetTexturePath().data() );
  pngReader->Update();
  vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
  texture->SetInputConnection(pngReader->GetOutputPort());

  model->set_mesh(obj_reader->GetOutput());
  model->set_texture(texture);
  ui->colorComboBox->setCurrentIndex(ui->colorComboBox->findText("RGB"));
  updateView();


  //get the normals the was read in the model
  //calculate the angle for each point
  // make a pcl and colorize

  //calculate the angle for each point
  // std::vector<double> normal_angles(model->normals.size());
  // for (size_t i = 0; i < model->normals.size(); i++) {
  //   double x1= model->normals[i][0];
  //   double y1=model->normals[i][1];
  //
  //   double x2=1.0;
  //   double y2=0.0;
  //
  //   double dot = x1*x2 + y1*y2;      //dot produt
  //   double det = x1*y2 - y1*x2;      //Determinant
  //   double angle = atan2(det, dot) ;  // atan2(y, x) or atan2(sin, cos)
  //
  //   angle = 0.0 + ((360.0 - 0.0) / (M_PI - -M_PI)) * (angle - -M_PI);
  //
  //   // angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);
  //
  //   // std::cout << "angle is " << angle << std::endl;
  //   normal_angles[i]=angle;
  // }
  //
  //
  //
  // //histogram it
  // double range=360.0;
  // double bucket_size=20.0;
  // int number_of_buckets = (int)ceil(range / bucket_size);
  // std::vector<int> histogram(number_of_buckets,0);
  //
  // for (size_t i = 0; i <   normal_angles.size(); i++) {
  //   int bucket = (int)floor(normal_angles[i] / bucket_size);
  //   histogram[bucket]++;
  // }
  //
  //
  // for (auto i: sort_indexes(histogram)) {
  //   std::cout << "hist: " << histogram[i] <<  " angle: " << i*bucket_size <<std::endl;
  // }
  //
  // int smallest_angle=0;
  // smallest_angle=sort_indexes(histogram)[0]*bucket_size;
  // std::cout << "smallest angle is" << smallest_angle << std::endl;
  //
  //
  //
  //
  //
  //
  //
  //
  // // Create the polydata where we will store all the geometric data
  // vtkSmartPointer<vtkPolyData> linesPolyData =
  //   vtkSmartPointer<vtkPolyData>::New();
  //
  //
  // // Create three points
  // double origin[3] = { 0.0, 0.0, 0.0 };
  // smallest_angle=sort_indexes(histogram)[0]*bucket_size;
  // double p0[3] = { 100* cos(smallest_angle * M_PI / 180.0), 100*sin(smallest_angle * M_PI / 180.0), 0.0 };
  // smallest_angle=sort_indexes(histogram)[1]*bucket_size;
  // double p1[3] = { 100* cos(smallest_angle * M_PI / 180.0), 100*sin(smallest_angle * M_PI / 180.0), 0.0 };
  // smallest_angle=sort_indexes(histogram)[2]*bucket_size;
  // double p2[3] = { 100* cos(smallest_angle * M_PI / 180.0), 100*sin(smallest_angle * M_PI / 180.0), 0.0 };
  // smallest_angle=sort_indexes(histogram)[3]*bucket_size;
  // double p3[3] = { 100* cos(smallest_angle * M_PI / 180.0), 100*sin(smallest_angle * M_PI / 180.0), 0.0 };
  // smallest_angle=sort_indexes(histogram)[4]*bucket_size;
  // double p4[3] = { 100* cos(smallest_angle * M_PI / 180.0), 100*sin(smallest_angle * M_PI / 180.0), 0.0 };
  // smallest_angle=sort_indexes(histogram)[5]*bucket_size;
  // double p5[3] = { 100* cos(smallest_angle * M_PI / 180.0), 100*sin(smallest_angle * M_PI / 180.0), 0.0 };
  //
  //
  //
  // draw_line(origin,p0);
  // draw_line(origin,p1);
  // draw_line(origin,p2);
  // draw_line(origin,p3);
  // draw_line(origin,p4);
  // draw_line(origin,p5);





  //REGION GROW
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //
  // // Generate pointcloud data
  // cloud->width = (model->points_wrapped.size());
  // cloud->height = 1;
  // cloud->points.resize (model->points_wrapped.size());
  //
  // for (size_t i = 0; i < cloud->points.size (); ++i)
  // {
  //   cloud->points[i].x = model->points_wrapped[i][0];
  //   cloud->points[i].y = model->points_wrapped[i][1];
  //   cloud->points[i].z = model->points_wrapped[i][2];
  // }
  //
  //
  // pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  // pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  // normals->width = (model->normals.size());
  // normals->height = 1;
  // normals->points.resize (model->normals.size());
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  // normal_estimator.setSearchMethod (tree);
  // normal_estimator.setInputCloud (cloud);
  // normal_estimator.setKSearch (50);
  // normal_estimator.compute (*normals);
  //
  //
  //
  // for (size_t i = 0; i < model->normals.size(); i++){
  //     normals->points[i].normal_x = model->normals[i][0];
  //     normals->points[i].normal_y = model->normals[i][1];
  //     normals->points[i].normal_z = model->normals[i][2];
  //
  //  }
  //
  //
  // // pcl::IndicesPtr indices (new std::vector <int>);
  // // pcl::PassThrough<pcl::PointXYZ> pass;
  // // pass.setInputCloud (cloud);
  // // pass.setFilterFieldName ("z");
  // // pass.setFilterLimits (0.0, 1.0);
  // // pass.filter (*indices);
  //
  // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  // reg.setMinClusterSize (50);
  // reg.setMaxClusterSize (1000000);
  // reg.setSearchMethod (tree);
  // reg.setNumberOfNeighbours (30);
  // reg.setInputCloud (cloud);
  // //reg.setIndices (indices);
  // reg.setInputNormals (normals);
  // reg.setSmoothnessThreshold (4.5 / 180.0 * M_PI);
  // // reg.setCurvatureThreshold (1.5);
  //
  // std::vector <pcl::PointIndices> clusters;
  // reg.extract (clusters);
  //
  //
  // std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  //
  // // std::cout << "These are the indices of the points of the initial" <<
  // //   std::endl << "cloud that belong to the first cluster:" << std::endl;
  // // int counter = 0;
  // // while (counter < clusters[0].indices.size ())
  // // {
  // //   std::cout << clusters[0].indices[counter] << ", ";
  // //   counter++;
  // //   if (counter % 10 == 0)
  // //     std::cout << std::endl;
  // // }
  // // std::cout << std::endl;
  //
  //
  // // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  // // pcl::visualization::CloudViewer viewer ("Cluster viewer");
  // // viewer.showCloud(colored_cloud);
  // // while (!viewer.wasStopped ())
  // // {
  // // }






  //OPENCV KMEANS CLUSTERING
  cv::Mat samples(model->normals.size(), 3, CV_32F);
  for( int y = 0; y < samples.rows; y++ ){
    for( int x = 0; x < samples.cols; x++ ){
      samples.at<float>(y,x)=model->normals[y][x];
    }
  }


  int clusterCount = 8;
  cv::Mat labels;
  int attempts = 10;
  cv::Mat centers;
  cv::kmeans(samples, clusterCount, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers );







  //mesh cluster
  // matrix_type_i colors;
  // colors.push_back(row_type_i {255,0,0});
  // colors.push_back(row_type_i {0,255,0});
  // colors.push_back(row_type_i {0,0,255});
  //
  // colors.push_back(row_type_i {255,255,0});
  // colors.push_back(row_type_i {255,0,255});
  // colors.push_back(row_type_i {0,255,255});
  //
  // colors.push_back(row_type_i {76,20,200});
  // colors.push_back(row_type_i {20,200,76});
  //
  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   //
  //  p_cloud->width    = model->points_wrapped.size();
  //  p_cloud->height   = 1;
  //  p_cloud->is_dense = false;
  //  p_cloud->points.resize (model->points_wrapped.size());
   //
   //
  //  for (size_t i = 0; i < model->points_wrapped.size(); i++){
  //     p_cloud->points[i].x = model->points_wrapped[i][0];
  //     p_cloud->points[i].y = model->points_wrapped[i][1];
  //     p_cloud->points[i].z = model->points_wrapped[i][2];
   //
  //     // p_cloud->points[i].r = normal_angles[i]*255;
  //     // p_cloud->points[i].r = interpolate (model->normals[i][0], -1.0, 1.0, 0.0, 255.0  );
  //     // p_cloud->points[i].g = interpolate (model->normals[i][1], -1.0, 1.0, 0.0, 255.0  );
  //     // p_cloud->points[i].b = interpolate (model->normals[i][2], -1.0, 1.0, 0.0, 255.0  );
   //
  //     int label=labels.at<int>(i,0);
   //
  //     // p_cloud->points[i].r = interpolate (label, 0.0, clusterCount, 0.0, 255.0  );
  //     p_cloud->points[i].r=colors[label][0];
  //     p_cloud->points[i].g=colors[label][1];
  //     p_cloud->points[i].b=colors[label][2];
   //
  //  }
   //
   //
  //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  //  viewer.showCloud (p_cloud);
  //  while (!viewer.wasStopped ())
  //  {
  //  }











  //Choose the bucket with the least angle. Rotate the mesh for that angle so that now the edge will be at 0.
  //repeat the process of calculating angles and histograming


  //mesh
  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   //
  //  p_cloud->width    = model->points_wrapped.size();
  //  p_cloud->height   = 1;
  //  p_cloud->is_dense = false;
  //  p_cloud->points.resize (model->points_wrapped.size());
   //
   //
  //  for (size_t i = 0; i < model->points_wrapped.size(); i++){
  //     p_cloud->points[i].x = model->points_wrapped[i][0];
  //     p_cloud->points[i].y = model->points_wrapped[i][1];
  //     p_cloud->points[i].z = model->points_wrapped[i][2];
   //
  //     // p_cloud->points[i].r = normal_angles[i]*255;
  //     p_cloud->points[i].r = interpolate (model->normals[i][0], -1.0, 1.0, 0.0, 255.0  );
  //     p_cloud->points[i].g = interpolate (model->normals[i][1], -1.0, 1.0, 0.0, 255.0  );
  //     p_cloud->points[i].b = interpolate (model->normals[i][2], -1.0, 1.0, 0.0, 255.0  );
   //
  //  }
   //
   //
  //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  //  viewer.showCloud (p_cloud);
  //  while (!viewer.wasStopped ())
  //  {
  //  }


  //normals
  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   //
  //  p_cloud->width    = model->normals.size();
  //  p_cloud->height   = 1;
  //  p_cloud->is_dense = false;
  //  p_cloud->points.resize (model->normals.size());
   //
   //
  //  for (size_t i = 0; i < model->normals.size(); i++){
  //     p_cloud->points[i].x = model->normals[i][0];
  //     p_cloud->points[i].y = model->normals[i][1];
  //     p_cloud->points[i].z = model->normals[i][2];
   //
  //     // p_cloud->points[i].r = normal_angles[i]*255;
  //     // p_cloud->points[i].g = 0;
  //     // p_cloud->points[i].b = 0;
  //  }
   //
  //  //add centers
  //  for (size_t c = 0; c < clusterCount; c++) {
  //    pcl::PointXYZRGB p;
  //    p.x=centers.at<float>(c,0);
  //    p.y=centers.at<float>(c,1);
  //    p.z=centers.at<float>(c,2);
  //    p.r=255;
  //    p_cloud->push_back(p);
  //  }
   //
   //
   //
  //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  //  viewer.showCloud (p_cloud);
  //  while (!viewer.wasStopped ())
  //  {
  //  }













  //segment and etract indicies

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //
  //  cloud->width    = model->points_wrapped.size();
  //  cloud->height   = 1;
  //  cloud->is_dense = false;
  //  cloud->points.resize (model->points_wrapped.size());
  //
  //
  //  for (size_t i = 0; i < model->points_wrapped.size(); i++){
  //     cloud->points[i].x = model->points_wrapped[i][0];
  //     cloud->points[i].y = model->points_wrapped[i][1];
  //     cloud->points[i].z = model->points_wrapped[i][2];
  //  }
  //
  // std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  // // for (size_t i = 0; i < cloud->points.size (); ++i)
  // //   std::cerr << "    " << cloud->points[i].x << " "
  // //                       << cloud->points[i].y << " "
  // //                       << cloud->points[i].z << std::endl;
  //
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // // Create the segmentation object
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.01);
  //
  // seg.setInputCloud (cloud);
  // seg.segment (*inliers, *coefficients);
  //
  // if (inliers->indices.size () == 0)
  // {
  //   PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  // }
  //
  // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
  //                                     << coefficients->values[1] << " "
  //                                     << coefficients->values[2] << " "
  //                                     << coefficients->values[3] << std::endl;
  //
  // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  // for (size_t i = 0; i < inliers->indices.size (); ++i)
  //   std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
  //                                              << cloud->points[inliers->indices[i]].y << " "
  //                                              << cloud->points[inliers->indices[i]].z << std::endl;


  //
  // pcl::ModelCoefficients plane_coeff;
  // plane_coeff.values.resize (4);
  // plane_coeff.values[0] = coefficients->values[0];
  // plane_coeff.values[1] = coefficients->values[1];
  // plane_coeff.values[2] = coefficients->values[2];
  // plane_coeff.values[3] = coefficients->values[3];
  // const std::string& id="";
  // int viewport;
  // pcl::visualization::PCLVisualizer::addPlane(plane_coeff, "plane",0);
  //
  // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
  // viewer.addPlane(plane_coeff, "p1");
  // viewer.addPointCloud(cloud, "c1");
  // // viewer.showCloud (cloud);
  //
  // while (!viewer.wasStopped ())
  // {
  // }














  //planar segmentation
  //create vector of clouds
  //create vector of planes

  //segment every cloud to get the plane coefficients
  //draw the planes

  //vector of clouds

  //Get number of elements in each cluster
  // std::vector<int> labels_vec;
  // for (size_t i = 0; i < labels.rows; i++) {
  //   labels_vec.push_back(labels.at<int>(i,0));
  // }
  //
  // std::vector<int> num_elements(clusterCount);
  // for (size_t i = 0; i < clusterCount; i++) {
  //   int num_elem = std::count (labels_vec.begin(), labels_vec.end(), i);
  //   num_elements[i]=num_elem;
  // }
  //
  // for (size_t i = 0; i < clusterCount; i++) {
  //   std::cout << "num elements in " << i << " is " <<  num_elements[i] << std::endl;
  // }

  //vector of clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds;
  for (size_t clust = 0; clust < clusterCount; clust++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    clustered_clouds.push_back(cloud);
  }

   for (size_t i = 0; i < model->points_wrapped.size(); i++){
      int label=labels.at<int>(i,0);
      pcl::PointXYZ p;
      p.x=model->points_wrapped[i][0];
      p.y=model->points_wrapped[i][1];
      p.z=model->points_wrapped[i][2];
      clustered_clouds[label]->push_back(p);
   }


   //vector of coefficients
   std::vector<pcl::ModelCoefficients> planes;
   for (size_t clust = 0; clust < clusterCount; clust++) {
     pcl::ModelCoefficients plane_coeff;
     plane_coeff.values.resize (4);
     planes.push_back(plane_coeff);
   }



   //for each cloud segment it
   for (size_t clust = 0; clust < clusterCount; clust++) {
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

     pcl::SACSegmentation<pcl::PointXYZ> seg;

     seg.setOptimizeCoefficients (true);
     // Mandatory
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setDistanceThreshold (0.01);

     seg.setInputCloud (clustered_clouds[clust]);
     seg.segment (*inliers, (planes[clust]));

     if (inliers->indices.size () == 0)
     {
       PCL_ERROR ("Could not estimate a planar model for the given dataset.");
     }

     std::cerr << "Model coefficients: " << planes[clust].values[0] << " "
                                          << planes[clust].values[1] << " "
                                          << planes[clust].values[2] << " "
                                          << planes[clust].values[3] << std::endl;

   }




   draw_sphere(0.0, 0.0, 0.0);




   //sort the planes by the angle between their normal and x axis
   //calculate the pairwise intersection line between them
   //calculate the angle of those lines to the x axis
   //for each point in the mesh get the two closest two lines
      // given the distances to them and their angles. Calculate the angle of the point. (coordinate 1)
      // calculate the distance of the points to the ransac plance we got earlier (coordinate 2)
      // coordinate 3 will be just the z position of the points.



   //Fix normals of planes so that they point towards the center
   //Dot product between the (point on a plane- center) and the normal (if it's negative then flip the normal)
   for (size_t i = 0; i < planes.size(); i++) {


     //WATCHOUT I SET Z and Y to 0 because if I set x and y Imight not get a solution since the plane might be paralel to Z axis.

     row_type dir(3);  //direction vector we get from a point on the plane and the center of mesh
     dir[0]= (- (planes[i].values[3] / planes[i].values[0]) ) - model->center[0];
     dir[1]= 0.0 - model->center[1];
     dir[2]= 0.0 - model->center[2];
     dir=normalize(dir);

     row_type normal(3);  //normal of the plane
     normal[0]=planes[i].values[0];
     normal[1]=planes[i].values[1];
     normal[2]=planes[i].values[2];
     normal=normalize(normal);

     double dot = normal[0] * dir[0]+
                  normal[1] * dir[1]+
                  normal[2] * dir[2];


     if (dot<0.0){
       std::cout << "flipping!!!!! " << i << std::endl;
       planes[i].values[0]=-planes[i].values[0];
       planes[i].values[1]=-planes[i].values[1];
       planes[i].values[2]=-planes[i].values[2];
       planes[i].values[3]=-planes[i].values[3];
     }
   }


   //get the angles
   std::vector<double> plane_angles(planes.size());
   for (size_t i = 0; i < plane_angles.size(); i++) {
       double x1= planes[i].values[0];
       double y1= planes[i].values[1];

       double x2=1.0;
       double y2=0.0;

       double dot = x1*x2 + y1*y2;      //dot produt
       double det = x1*y2 - y1*x2;      //Determinant
       double angle = atan2(det, dot) ;  // atan2(y, x) or atan2(sin, cos)

      //  angle = 0.0 + ((360.0 - 0.0) / (M_PI - -M_PI)) * (angle - -M_PI);

       angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);
      //  angle = interpolate ( angle , 0.5, 0.9, 0.0, 1.0);

      //  std::cout << "angle is " << angle << std::endl;
       plane_angles[i]=angle;
   }


   //order the planes with respect to the angles
   //Need to establish some relation between them so we make a new vector
   std::vector<plane_struct> planes_ordered(planes.size());

   //fill it
   for (size_t i = 0; i < planes.size(); i++) {
     planes_ordered[i].coef=planes[i];
     planes_ordered[i].angle=plane_angles[i];
   }
   std::sort(planes_ordered.begin(), planes_ordered.end(), by_angle());





   //go though every plane pair and get the line between them
   //https://www.mathworks.com/matlabcentral/fileexchange/17618-plane-intersection/content/plane_intersect.m
   std::vector <line_struct> lines(planes_ordered.size());
   for (size_t i = 0; i < planes_ordered.size(); i++) {
     int curr = i;
     int next = i +1;
     if (next >= planes_ordered.size()) {
       next=0;
     }

     std::cout << "intersect betwen " << curr << " " << next  << std::endl;
     //need the direction vetor which is the cross product between the normals of the two planes_ordered
     row_type N_1(3);
     row_type N_2(3);

     N_1[0]=planes_ordered[curr].coef.values[0];
     N_1[1]=planes_ordered[curr].coef.values[1];
     N_1[2]=planes_ordered[curr].coef.values[2];

     N_2[0]=planes_ordered[next].coef.values[0];
     N_2[1]=planes_ordered[next].coef.values[1];
     N_2[2]=planes_ordered[next].coef.values[2];

     row_type A1(3);  //Point that belong on plane 1
     row_type A2(3);  //Point that belong on plane 2

     A1[0]=(- (planes[curr].values[3] / planes[curr].values[0]) );
     A1[1]=0.0;
     A1[2]=0.0;

     A2[0]=(- (planes[next].values[3] / planes[next].values[0]) );
     A2[1]=0.0;
     A2[2]=0.0;


     row_type P(3);  //Point on the line that lies on the intersection
     row_type N(3);  //direction vector of the intersect line

     //  N=cross(N1,N2);
     N[0]= N_1[1]*N_2[2] - N_1[2]*N_2[1];
     N[1]= N_1[2]*N_2[0] - N_1[0]*N_2[2];
     N[2]= N_1[0]*N_2[1] - N_1[1]*N_2[0];

     std::cout << "N is " << N[0] << " " << N[1] << " " << N[2] << std::endl;

     double d1=planes_ordered[curr].coef.values[3]; //constant of plane 1
     double d2=planes_ordered[next].coef.values[3]; //constant of plane 2


     //find the maximum coordinate in the N vector and 0 that one
     int maxc=std::distance(N.begin(), std::max_element(N.begin(), N.end(), abs_compare));
     std::cout << "maxc is" << maxc << std::endl;

     //TODO:Fill the other cases.
     switch (maxc) {
       case 0:

       break;

       case 1:
       break;

       case 2:

       break;
     }

     P[0]=(d2*N_1[1] - d1*N_2[1]) / N[2];
     P[1]=(d1*N_2[0] - d2*N_1[0]) / N[2];
     P[2]=0.0;

     lines[i].direction=N;
     lines[i].point=P;

     std::cout << "point of intersect is" << P[0] << " " << P[1] << " " << P[2] << std::endl;

     draw_sphere(P[0], P[1], P[2] );


   }

   //Calculate the angle of the lines
   for (size_t i = 0; i < lines.size(); i++) {
       double x1= lines[i].point[0];
       double y1= lines[i].point[1];

       double x2=1.0;
       double y2=0.0;

       double dot = x1*x2 + y1*y2;      //dot produt
       double det = x1*y2 - y1*x2;      //Determinant
       double angle = atan2(det, dot) ;  // atan2(y, x) or atan2(sin, cos)

      //  angle = 0.0 + ((360.0 - 0.0) / (M_PI - -M_PI)) * (angle - -M_PI);

       angle = interpolate ( angle , -M_PI, M_PI, 0.0, 1.0);
      //  angle = interpolate ( angle , 0.5, 0.9, 0.0, 1.0);

       std::cout << "angle of line is " << angle << std::endl;
      lines[i].angle=angle;
   }

   //sort them to assign indices
   std::sort(lines.begin(),lines.end(),by_angle_line());
   for (size_t i = 0; i < lines.size(); i++) {
     lines[i].index=i;
   }


   std::cout << "Starting to calculate actual angle for all the points" << std::endl;
   std::vector<double> test_angles;
   std::vector<double> test_distances;
   for (size_t i = 0; i < model->points_wrapped.size(); i++) {
     //calculate the distance to all the lines
     //Sort the lines vector by distance
     //grab the 2 closest lines
     //calculate the distance between the points of those two lines
     // with the angle of the 2 closest lines and the distance to them and betweem them we get the angle of the mesh point

     //get the label of the mesh point (from the previous kmeans segmentation)
     //from that label we know the index of the plane asociated
     //calculate distance from point to plane


     //http://gamedev.stackexchange.com/questions/72528/how-can-i-project-a-3d-point-onto-a-3d-line
     //A + dot(AP,AB) / dot(AB,AB) * AB


     for (size_t line_idx = 0; line_idx < lines.size(); line_idx++) {
      //  row_type AP, AB, inter;  //AB is the normal vector of the line, AP is the vector between the mesh point the line.point
      //  double dot_ap_ab;
      //  double dot_ab_ab;
       //
      //  AP[0]=points_wrapped[i][0] -  line[line_idx].point[0];
      //  AP[1]=points_wrapped[i][1] -  line[line_idx].point[1];
      //  AP[2]=points_wrapped[i][2] -  line[line_idx].point[2];
       //
      //  inter[0]=line[line_idx].point[0] + dot (AP,AB) / dot (AB,AB) * line[line_idx].normal[0];
      //  inter[1]=line[line_idx].point[1] + dot (AP,AB) / dot (AB,AB) * line[line_idx].normal[1];
      //  inter[2]=line[line_idx].point[2] + dot (AP,AB) / dot (AB,AB) * line[line_idx].normal[2];
       //
      //  //now calculate distance between

      //http://geomalgorithms.com/a02-_lines.html
      row_type w(3); //w is mesh_point-line.point;
      w[0]=model->points_wrapped[i][0] -  lines[line_idx].point[0];
      w[1]=model->points_wrapped[i][1] -  lines[line_idx].point[1];
      w[2]=model->points_wrapped[i][2] -  lines[line_idx].point[2];

      row_type u_l= normalize (lines[line_idx].direction);

      row_type cross_res;
      cross_res= cross(u_l,w);


      double distance= norm(cross_res);
      lines[line_idx].distance=distance;


     }

     //sort the lines by distance
     std::sort(lines.begin(), lines.end(), by_distance());
     //
    //  std::cout << "dist of 1 " <<  lines[0].distance  << std::endl;
    //  std::cout << "dist of 2 " <<  lines[1].distance  << std::endl;
    //  std::cout << "dist of 3 " <<  lines[2].distance  << std::endl;
    //  std::cout << "/* message */" << std::endl;
    //  std::cout << "/* message */" << std::endl;

     double dist_total;
     dist_total=lines[0].distance + lines[1].distance;

     double angle=0.0;
     angle=  (lines[0].distance/ dist_total) * lines[1].angle + (lines[1].distance/ dist_total) * lines[0].angle;

     //need to take into account if the point is between face 7 and 0. Then face 0 needs to be considered at angle 1.0 not 0.0
     if (lines[0].index==0 && lines[1].index==lines.size()-1){
       angle=  (lines[0].distance/ dist_total) * lines[1].angle + (lines[1].distance/ dist_total) * 1.0;
     }

     if (lines[1].index==0 && lines[0].index==lines.size()-1){
       angle=  (lines[0].distance/ dist_total) * 1.0 + (lines[1].distance/ dist_total) * lines[0].angle;
     }

    //  std::cout << "angle is " << angle << std::endl;
    test_angles.push_back(angle);



    //calculate distance to plane
    int label=labels.at<int>(i,0);

    //http://mathinsight.org/distance_point_plane
    // double numerator;
    // double denominator;
    //
    //
    // numerator= abs( planes[label].values[0] * model->points_wrapped[i][0] +
    //                 planes[label].values[1] * model->points_wrapped[i][1] +
    //                 planes[label].values[2] * model->points_wrapped[i][2] +
    //                 planes[label].values[3] );
    //
    // denominator=sqrt( planes[label].values[0]*planes[label].values[0] +
    //                   planes[label].values[1]*planes[label].values[1] +
    //                   planes[label].values[2]*planes[label].values[2]);
    //
    // // std::cout << "numerator " << numerator << std::endl;
    // // std::cout << "denominator " << denominator << std::endl;
    //
    // double distance= numerator/denominator;
    //
    // // std::cout << "distance is " << distance  << std::endl;



    //http://mathworld.wolfram.com/HessianNormalForm.html
    double p;
    p = planes[label].values[3]/ sqrt( planes[label].values[0]*planes[label].values[0] +
                                       planes[label].values[1]*planes[label].values[1] +
                                       planes[label].values[2]*planes[label].values[2]);


    double dot_res;
    row_type normal(3);
    normal[0]=planes[label].values[0];
    normal[1]=planes[label].values[1];
    normal[2]=planes[label].values[2];

    dot_res=dot(normal,model->points_wrapped[i]);

    double distance = dot_res + p;

    //TODO: Remove this, only added for debugging purposes
    if (fabs(distance) > 0.5){
      // std::cout << "slammmmm" << std::endl;
      distance=0.0;
    }

    // std::cout << "distance is " << distance  << std::endl;
    test_distances.push_back(distance);



   }



   //estimate circumference as the biggest distance between the center and line.
   // if the mesh is centerred then he center is 0
   double radius=-9999;
   for (size_t line_idx = 0; line_idx < lines.size(); line_idx++) {
     row_type w(3); //w is mesh_point-line.point;
     w[0]=0.0 -  lines[line_idx].point[0];
     w[1]=0.0 -  lines[line_idx].point[1];
     w[2]=0.0 -  lines[line_idx].point[2];

     row_type u_l= normalize (lines[line_idx].direction);

     row_type cross_res;
     cross_res= cross(u_l,w);

     double distance= norm(cross_res);
     if (distance>radius){
       radius=distance;
     }
   }
   std::cout << "estimated radius is " << radius  << std::endl;
   double circumference=2*M_PI*radius;






   //also need to rotate the normals taking into account the normals of the planes




   //brute forcing and writing directly into the model
   model->points_unwrapped.resize(model->num_points);
   for (size_t i = 0; i < model->num_points; i++) {
     model->points_unwrapped[i].resize(model->point_components);
     model->points_unwrapped[i][0]=test_angles[i] *circumference;
     model->points_unwrapped[i][1]=test_distances[i];
     model->points_unwrapped[i][2]=model->points_wrapped[i][2];
   }







   //PCL mesh with angles
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //
    // p_cloud->width    = model->points_wrapped.size();
    // p_cloud->height   = 1;
    // p_cloud->is_dense = false;
    // p_cloud->points.resize (model->points_wrapped.size());
    //
    //
    // for (size_t i = 0; i < model->points_wrapped.size(); i++){
    //   //  p_cloud->points[i].x = model->points_wrapped[i][0];
    //   //  p_cloud->points[i].y = model->points_wrapped[i][1];
    //   //  p_cloud->points[i].z = model->points_wrapped[i][2];
    //
    //   p_cloud->points[i].x = test_angles[i] *circumference;
    //   p_cloud->points[i].y = test_distances[i];
    //   p_cloud->points[i].z = model->points_wrapped[i][2];
    //
    //
    //
    //    // p_cloud->points[i].r = normal_angles[i]*255;
    //   //  p_cloud->points[i].r = test_angles[i]*255;
    //   //  p_cloud->points[i].g = 0.0;
    //   //  p_cloud->points[i].b = 0.0;
    //
    //   p_cloud->points[i].r = interpolate (model->normals[i][0], -1.0, 1.0, 0.0, 255.0  );
    //   p_cloud->points[i].g = interpolate (model->normals[i][1], -1.0, 1.0, 0.0, 255.0  );
    //   p_cloud->points[i].b = interpolate (model->normals[i][2], -1.0, 1.0, 0.0, 255.0  );
    //
    // }
    //
    //
    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer2");
    // viewer.showCloud (p_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }












   for (size_t clust = 0; clust < clusterCount; clust++) {
     draw_plane(planes_ordered[clust].coef.values[0],
                planes_ordered[clust].coef.values[1],
                planes_ordered[clust].coef.values[2],
                planes_ordered[clust].coef.values[3],
                planes_ordered[clust].angle,0.0,0.0);
   }





  int num_actors= renderer->GetActors()->GetNumberOfItems();
  std::cout << "num of actors" << num_actors << std::endl;
}



row_type Visualizer::normalize (row_type vec){
  double length=0.0;
  for (size_t i = 0; i < vec.size(); i++) {
      length += vec[i]*vec[i];
  }
  length=sqrt(length);

  for (size_t i = 0; i < vec.size(); i++) {
      vec[i]=vec[i]/length;
  }

  return vec;

}


double Visualizer::dot (row_type vec1, row_type vec2){
  double ret=0.0;
  for (size_t i = 0; i < vec1.size(); i++) {
    ret+=vec1[i]*vec2[i];
  }
  return ret;
}

row_type Visualizer::cross (row_type N_1, row_type N_2){
  row_type N(3);
  N[0]= N_1[1]*N_2[2] - N_1[2]*N_2[1];
  N[1]= N_1[2]*N_2[0] - N_1[0]*N_2[2];
  N[2]= N_1[0]*N_2[1] - N_1[1]*N_2[0];
  return N;
}

double Visualizer::norm (row_type vec){
  double length=0.0;
  for (size_t i = 0; i < vec.size(); i++) {
      length += vec[i]*vec[i];
  }
  length=sqrt(length);
  return length;
}



void Visualizer::draw_line(double* p0,double* p1){

  vtkSmartPointer<vtkLineSource> lineSource =
    vtkSmartPointer<vtkLineSource>::New();
  lineSource->SetPoint1(p0);
  lineSource->SetPoint2(p1);
  lineSource->Update();

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(lineSource->GetOutputPort());
  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(4);

  renderer->AddActor(actor);

}


void Visualizer::draw_plane(double c0,double c1, double c2, double c3, double r, double g, double b){

  std::cout << "creating plane" << std::endl;

  // Create a plane
  vtkSmartPointer<vtkPlaneSource> planeSource =
    vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->SetCenter(0.0, 0.0, 0.0);
  planeSource->SetNormal(c0, c1, c2);
  planeSource->Push(-c3);
  planeSource->Update();

  vtkPolyData* plane = planeSource->GetOutput();

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();

  mapper->SetInputData(plane);

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(r, g, b); //(R,G,B)
  // actor->GetProperty()->BackfaceCullingOn();

    renderer->AddActor(actor);



}


void  Visualizer::draw_sphere(double x, double y, double z){

  // Create a sphere
    vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(x, y, z);
    sphereSource->SetRadius(0.1);

    vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> actor =
      vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    renderer->AddActor(actor);

}




double Visualizer::interpolate ( double input , double input_start, double input_end, double output_start, double output_end){

  double output;
  output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);

  return output;

}









void Visualizer::clearAll(){
  model->clear();
  renderer->RemoveAllViewProps();
}


void  Visualizer::updateView(int reset_camera){
  std::cout << "update view" << std::endl;
  renderer->RemoveAllViewProps();

  //update the wall with the new points (wrapped on unwrapped)
  vtkSmartPointer<vtkPolyData> wall=model->get_mesh();

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(wall);
  mapper->Update();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->SetTexture(model->m_full_texture);
  actor->GetProperty()->BackfaceCullingOn();

  renderer->AddActor(actor);
  if (reset_camera==1){
    renderer->ResetCamera();
  }


  this->ui->qvtkWidget->GetRenderWindow()->Render();

}


void Visualizer::on_clearButton_clicked(){
  std::cout << "clearing the view" << std::endl;
  renderer->RemoveAllViewProps();
  this->ui->qvtkWidget->GetRenderWindow()->Render();
}

void Visualizer::on_unwrapButton_clicked(){
  std::cout << "unwrapping" << std::endl;


  if (!model->points_wrapped.empty()){
    model->is_unwrapped=!model->is_unwrapped;
  }

  if (model->points_unwrapped.empty() && !model->points_wrapped.empty()){
    // model->compute_unwrap();
    model->compute_unwrap2();
  }

  updateView();
}

void Visualizer::on_colorComboBox_currentIndexChanged(const QString & text){
  std::cout << text.toStdString() << std::endl;

  if (text.toStdString() == "Plain"){
    std::cout << "calculating plain colors" << std::endl;
    model->compute_plain_colors();
  }
  if (text.toStdString() == "RGB"){
    std::cout << "calculating RGB colors" << std::endl;
    model->compute_rgb_colors();
  }
  if (text.toStdString() == "Depth"){
    std::cout << "calculating Depth colors" << std::endl;
    model->compute_depth_colors();
  }
  if (text.toStdString() == "Curvature"){
    std::cout << "calculating Curvature colors" << std::endl;
  }

  updateView(0);  //by passing 0 it forces the camera to not be reset when updating the renderer view

}

void Visualizer::on_perspectiveCheckBox_clicked(){
  std::cout << "change perspective" << std::endl;

  int current_val=renderer->GetActiveCamera()->GetParallelProjection();
  if (current_val==0)
    renderer->GetActiveCamera()->SetParallelProjection(1);
  else
    renderer->GetActiveCamera()->SetParallelProjection(0);

  updateView(0);  //by passing 0 it forces the camera to not be reset when updating the renderer view

}

void Visualizer::on_selectButton_clicked(){
  std::cout << "selecting defects" << std::endl;
  model->selecting_defects=!model->selecting_defects;
}

void Visualizer::on_gridButton_clicked(){
  std::cout << "grid mode" << std::endl;
  model->selecting_grid=!model->selecting_grid;


  renderer->RemoveAllViewProps();
  renderer->ResetCamera();




  //unwrapping
  if (!model->points_wrapped.empty()){
    model->is_unwrapped=true;
  }
  if (model->points_unwrapped.empty() && !model->points_wrapped.empty()){
    model->compute_unwrap();
  }


  //at this point we have the unwrapped points, but they are not yet set to the actuall wall
  //we call the get mesh, to write the point_unwrapped into the actual wall (don't now if necesary)
  model->get_mesh();

  //forming a grid
  model->create_grid();
  std::cout << "visualizer finished creating grid" << std::endl;

  //iterating through all the grid cells polydata and make actors for them
  for (size_t i = 0; i < model->grid_cells.size(); i++) {
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(model->grid_cells[i]);
    //mapper->ImmediateModeRenderingOn();
    mapper->StaticOn();
    //mapper->Update();
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    renderer->AddActor(actor);
    std::cout << "added actor nr " << i << std::endl;
  }


  this->ui->qvtkWidget->GetRenderWindow()->Render();
  //updateView();

}

void Visualizer::on_updateViewButton_clicked(){
  std::cout << "updating iew" << std::endl;
  updateView();
}


void Visualizer::slotExit()
{
  qApp->exit();
}
