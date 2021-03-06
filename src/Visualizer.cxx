#include "Visualizer.h"

// This is included here because it is forward declared in
// Visualizer.h
#include "ui_Visualizer.h"

#define MAGNIFICATION 5


// template <typename T>
// std::vector<size_t> sort_indexes(const std::vector<T> &v) {
//
//   // initialize original index locations
//   std::vector<size_t> idx(v.size());
//   iota(idx.begin(), idx.end(), 0);
//
//   // sort indexes based on comparing values in v
//   sort(idx.begin(), idx.end(),
//        [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});
//
//   return idx;
// }




Visualizer::Visualizer():
  model(new Model()),
  renderer(vtkSmartPointer<vtkRenderer>::New()),
  interactor(vtkSmartPointer<InteractorPointPicker>::New()),
  m_grid_metric_actor(vtkSmartPointer<vtkTextActor>::New()),
  m_mapper_wall(vtkSmartPointer<vtkPolyDataMapper>::New()),
  m_actor_wall(vtkSmartPointer<vtkActor>::New()),
  m_config(new ConfigDialog(this))
{
  this->ui = new Ui_Visualizer;
  this->ui->setupUi(this);

  this->ui->colorComboBox->addItem("Plain");
  this->ui->colorComboBox->addItem("RGB (bright)");
  this->ui->colorComboBox->addItem("RGB (original)");
  this->ui->colorComboBox->addItem("IR");
  this->ui->colorComboBox->addItem("Depth");
  this->ui->colorComboBox->addItem("Depth (colored)");
  this->ui->colorComboBox->addItem("Depth (defects)");
  //this->ui->colorComboBox->addItem("")
  // this->ui->colorComboBox->addItem("Curvature");

  this->ui->numWallsText->setText(QString::number(model->m_num_walls));


  this->ui->loadFileConfigButton->setIcon(QIcon("../cog_1.png"));
  this->ui->loadFileConfigButton->setIconSize(QSize(26,26));

  m_grid_actors.clear();

  // //NEW UI
  // QGridLayout *layout = new QGridLayout;
  // QWidget * central = new QWidget();
  // setCentralWidget(central);
  //
  // // QLabel *lbl = new QLabel;
  // // QMovie *movie = new QMovie("/media/alex/Data/Master/SHK/vtk_scripts/gifs/spinner.gif");
  // // lbl->setMovie(movie);
  // // lbl->show();
  // // lbl->setWindowFlags(Qt::FramelessWindowHint);
  // // lbl->setStyleSheet("background-color: rgba(225,255,255,0);");
  // // lbl->setAttribute( Qt::WA_TranslucentBackground, true );
  // // movie->start();
  // //
  // // lbl->hide();
  // // lbl->show();
  // // lbl->raise();
  //
  // QVTKWidget *qvtkWidget = new QVTKWidget;
  //
  // // layout->addWidget(lbl);
  // layout->addWidget(qvtkWidget);
  //
  // centralWidget()->setLayout(layout);
  //
  // setWindowTitle("Camera Window");
  // // setFixedSize(1000, 800);
  //
  //
  // renderer->GradientBackgroundOn();
  // renderer->SetBackground(1.0,1.0,1.0);
  // renderer->SetBackground2(0.1,0.1,0.1);
  //
  // qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
  //
  //
  // // QPushButton* _widgetOnTheTop = new QPushButton(this);
  // //     _widgetOnTheTop->setGeometry(10,10,100,35);
  // //     _widgetOnTheTop->show();
  //
  //
  //
  //
  //
  //
  //
  // QLabel *lbl = new QLabel(this);
  // QMovie *movie = new QMovie("/media/alex/Data/Master/SHK/vtk_scripts/gifs/spinner.gif");
  // lbl->setMovie(movie);
  // lbl->setWindowFlags(Qt::FramelessWindowHint);
  // lbl->setStyleSheet("background-color: rgba(225,255,255,0);");
  // lbl->setStyleSheet("background-color: rgba(0,0,0,0%)");
  //
  // // lbl->setAttribute( Qt::WA_TranslucentBackground, true );
  // lbl->setAttribute(Qt::WA_NoSystemBackground, true);
  //
  // lbl->setGeometry(100,100,666,666);
  // lbl->show();
  // movie->start();
  //



  // VTK Renderer
  renderer->GradientBackgroundOn();
  renderer->SetBackground(1.0,1.0,1.0);
  renderer->SetBackground2(0.1,0.1,0.1);
  m_actor_wall->SetMapper(m_mapper_wall);
  m_actor_wall->GetProperty()->BackfaceCullingOff();
  renderer->AddActor(m_actor_wall);



  // Changing interactor to a custom one to handle different mouse events
  vtkSmartPointer<vtkPointPicker> pointPicker = vtkSmartPointer<vtkPointPicker>::New();
  // vtkSmartPointer<vtkCellPicker> pointPicker = vtkSmartPointer<vtkCellPicker>::New();
  this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle( interactor );
  this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(pointPicker);
  interactor->selecting_defects=&(model->m_selecting_defects);



  renderer->GetActiveCamera()->SetParallelProjection(0);

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

  // Set up action signals and slots
  connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

  connect(interactor, SIGNAL(right_click_pressed_signal(row_type )  ),
          model.get(), SLOT(right_click_pressed_slot(row_type)  ));
  connect(model.get(), SIGNAL(grid_changed_signal()),
          this, SLOT(grid_changed_slot()) );


  #if VTK_MAJOR_VERSION <= 5
      std::cout << "vtk version lower than 5" << std::endl;
  #else
      std::cout << "vtk verson higher than 5" << std::endl;
  #endif

}



void Visualizer::on_loadFileButton_clicked(){


  QString file_name;
  QString selfilter = tr("Mesh (*.obj *.ply)");
  file_name = QFileDialog::getOpenFileName(this, tr("Open File"), "./", selfilter);
  if (file_name.isEmpty()){
    return;
  }

  // QString file_name="/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/researchDenslyTexturedMesh.obj";
  // QString file_name="/media/alex/Data/Master/SHK/Data/New_data/ply_3/optim_colored_o4.ply";
  // QString file_name="/media/alex/Data/Master/SHK/Data/euroc/mve_scene_inpainted/surface-L2-clean_crop2.ply";
  // QString file_name="/media/alex/Data/Master/SHK/Data/euroc/mve_scene_inpainted/surface-L2-clean.ply";

  std::cout << "filename: " << file_name.toStdString() << std::endl;


  model->mesh->clear();
  model->mesh->set_nr_walls(  atoi (this->ui->numWallsText->text().toStdString().data())  ) ;

  if (boost::ends_with(file_name.toStdString(), ".ply")) {
    std::cout << "reading .ply file" << std::endl;

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName ( file_name.toStdString().data() );
    reader->Update();

    model->mesh->set_state_ply(true);
    model->mesh->set_mesh(reader->GetOutput());

  }else if (boost::ends_with(file_name.toStdString(), ".obj")){
    std::cout << "reading .obj file" << std::endl;

    std::unique_ptr<OBJReader2> obj_reader(new OBJReader2());
    obj_reader->experimental_loading=model->m_experiemental_loading;
    obj_reader->SetFileName(file_name.toStdString());
    obj_reader->Update();

    //brightned texture
    vtkSmartPointer<vtkPNGReader> pngReader_bright = vtkSmartPointer<vtkPNGReader>::New();
    pngReader_bright->SetFileName (obj_reader->GetTexturePath().data() );
    pngReader_bright->Update();
    vtkSmartPointer<vtkTexture> texture_bright = vtkSmartPointer<vtkTexture>::New();
    texture_bright->SetInputConnection(pngReader_bright->GetOutputPort());
    // texture_bright->Update();

    //original texture
    vtkSmartPointer<vtkPNGReader> pngReader_orig = vtkSmartPointer<vtkPNGReader>::New();
    pngReader_orig->SetFileName (obj_reader->GetTextureOriginalPath().data() );
    pngReader_orig->Update();
    vtkSmartPointer<vtkTexture> texture_orig = vtkSmartPointer<vtkTexture>::New();
    texture_orig->SetInputConnection(pngReader_orig->GetOutputPort());
    // texture_orig->Update();


    model->mesh->set_state_obj(true);
    model->mesh->set_mesh(obj_reader->GetOutput());
    model->mesh->set_texture_bright(texture_bright);
    model->mesh->set_texture_original(texture_orig);

  }else{
    std::cout << "NOT VALID FORMAT" << std::endl;
    return;
  }





  std::cout << "before setting colors we have a mesh with points " << model->mesh->get_nr_points_wrapped() << '\n';


  ui->colorComboBox->setCurrentIndex(ui->colorComboBox->findText("RGB (bright)"));
  on_colorComboBox_currentIndexChanged("RGB (bright)");  //Explicitly call the function to recompute the colors in case the previous line didn't actualy change anything and therefore the slot didn't fire

  std::cout << "after setting colors we have a mesh with points " << model->mesh->get_nr_points_wrapped() << '\n';

  updateView();


  std::cout << "finished--------------------------" << '\n';








//   QString file_name;
//   QString selfilter = tr("Mesh (*.obj *.ply)");
//   file_name = QFileDialog::getOpenFileName(this,
//   	         tr("Open File"), "./", selfilter);
//
//   if (file_name.isEmpty()){
//     return;
//   }
//
//   // QString file_name="/media/alex/Data/Master/SHK/Data/Chimney/research_textured_mesh/researchDenslyTexturedMesh.obj";
//
//   std::cout << "filename: " << file_name.toStdString() << std::endl;
//
//
//   if (boost::ends_with(file_name.toStdString(), ".ply")) {
//     std::cout << "reading .ply file" << std::endl;
//
//
//
//     vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
//     reader->SetFileName ( file_name.toStdString().data() );
//     reader->Update();
//
//     vtkSmartPointer<vtkPolyData> mesh_orientated= auto_fix_orientation(reader->GetOutput());
//
//
//     //Get the colors
//     //Brighten them
//     //write them in the mesh orientated, they will atuometically get read by the set mesh
//
//     //The original one, brutce forcefully set them inside the model
//
//
//     auto rgb_unaltered  =mesh_orientated->GetPointData()->GetScalars();
//     vtkSmartPointer<vtkUnsignedCharArray> rgb_bright= vtkSmartPointer<vtkUnsignedCharArray>::New();
//     rgb_bright->SetNumberOfComponents(3);
//
//     int num_colors=rgb_unaltered->GetNumberOfTuples();
//
//     cv::Mat colors_bright(num_colors, 1,  CV_8UC3);
//
//     for (size_t i = 0; i < num_colors; i++) {
//       cv::Vec3b &intensity = colors_bright.at<cv::Vec3b>(i, 0);
//
//       double* color_pixel;
//       color_pixel=rgb_unaltered->GetTuple(i);
//
//       intensity.val[0] = color_pixel[0];
//       intensity.val[1] = color_pixel[1];
//       intensity.val[2] = color_pixel[2];
//     }
//
//
//
//
//     //brithen colors bright
//     cv::Mat hsv;
//     cv::cvtColor(colors_bright,hsv,CV_BGR2HSV);
//     std::vector<cv::Mat> channels;
//     cv::split(hsv,channels);
//     cv::normalize(channels[1], channels[1], 0, 255, cv::NORM_MINMAX);
//     cv::normalize(channels[2], channels[2], 0, 255, cv::NORM_MINMAX);
//     cv::Mat result;
//     cv::merge(channels,hsv);
//     cv::cvtColor(hsv,result,CV_HSV2BGR);
//     result.copyTo(colors_bright);
//
//     //Copy colors bright back into a unsigned char arrays
//     for (int i = 0; i < colors_bright.cols; i++) {
//       for (int j = 0; j < colors_bright.rows; j++) {
//           cv::Vec3b intensity = colors_bright.at<cv::Vec3b>(j, i);
//
//           double color_pixel[3];
//           color_pixel[0]= intensity.val[0];
//           color_pixel[1]= intensity.val[1];
//           color_pixel[2]= intensity.val[2];
//
//           rgb_bright->InsertNextTuple(color_pixel);
//       }
//     }
//
//
//       mesh_orientated->GetPointData()->SetScalars(rgb_bright);
//
// //-----------
//
//     model->set_mesh(mesh_orientated);
//     model->m_colors_original=vtkUnsignedCharArray::SafeDownCast(rgb_unaltered);
//     model->m_colors_bright=vtkUnsignedCharArray::SafeDownCast(rgb_bright);
//
//     model->m_is_ply=true;  //Need to add this at the finale because set mesh reset everything to defualts
//     model->m_is_obj=false;
//
//     //copy the ply to a temp_ply
//     //decimate the temp
//     //Get normals for it (polydata normals)
//     // For each point in the original ply grab the neares K poin in the temp one
//     //Normals of the original ply will be a weigther average (wight=distance) of the normals of the neightbours
//     //then it would be no need to to blur normals in the case that we read a ply file
//
//
//
//   }else if (boost::ends_with(file_name.toStdString(), ".obj")){
//     std::cout << "reading .obj file" << std::endl;
//
//     std::unique_ptr<OBJReader2> obj_reader(new OBJReader2());
//     obj_reader->experimental_loading=model->m_experiemental_loading;
//     obj_reader->should_fix_orientation=model->m_fix_orientation;
//     obj_reader->SetFileName(file_name.toStdString());
//     obj_reader->Update();
//
//     //brightned texture
//     vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
//     pngReader->SetFileName (obj_reader->GetTexturePath().data() );
//     pngReader->Update();
//     vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
//     texture->SetInputConnection(pngReader->GetOutputPort());
//
//     //original texture
//     vtkSmartPointer<vtkPNGReader> pngReader_orig = vtkSmartPointer<vtkPNGReader>::New();
//     pngReader_orig->SetFileName (obj_reader->GetTextureOriginalPath().data() );
//     pngReader_orig->Update();
//     vtkSmartPointer<vtkTexture> texture_orig = vtkSmartPointer<vtkTexture>::New();
//     texture_orig->SetInputConnection(pngReader_orig->GetOutputPort());
//
//     //Fix orientation
//     vtkSmartPointer<vtkPolyData> mesh_orientated= auto_fix_orientation(obj_reader->GetOutput());
//
//     model->set_mesh(mesh_orientated);
//     model->set_texture_bright(texture);
//     model->set_texture_original(texture_orig);
//
//     vtkSmartPointer<vtkDataArray> tcoords = obj_reader->GetOutput()->GetPointData()->GetTCoords();
//     if(tcoords){
//       std::cout << "RGB:the polydata has tcoords" << std::endl;
//     }else{
//       std::cout << "RGB:the polydata does not have tcoords" << std::endl;
//     }
//     model->tcoords_rgb=tcoords;
//
//     std::cout << "visualizer:: rgb toocrds numer: "  << model->tcoords_rgb->GetSize()<< std::endl;
//
//     model->m_is_obj=true;
//     model->m_is_ply=false;
//   }else{
//     std::cout << "NOT VALID FORMAT" << std::endl;
//     return;
//   }
//
//
//
//   ui->colorComboBox->setCurrentIndex(ui->colorComboBox->findText("RGB (bright)"));
//   updateView();

}


vtkSmartPointer<vtkPolyData> Visualizer::auto_fix_orientation( vtkSmartPointer<vtkPolyData> polydata){
  std::cout << "fixing orientation" << std::endl;

  matrix_type normals_for_orientation;
  int downsample=10;

  double angle=0.0;

  vtkSmartPointer<vtkFloatArray> vtk_normals = vtkSmartPointer<vtkFloatArray>::New();
  vtk_normals->SetNumberOfComponents(3);
  vtk_normals->SetName("Normals");
  vtk_normals = vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());

  if (vtk_normals){
    std::cout << "autofix:: yes it has normals" << std::endl;
    std::cout << "autofix nr of normals" << vtk_normals->GetNumberOfTuples() << std::endl;
    normals_for_orientation=vtk_normal_tcoords_to_vector(vtk_normals);
  }else{
    std::cout << "autofix: it doesnt have normals" << std::endl;

    //We will estmate them

    vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
    #if VTK_MAJOR_VERSION <= 5
      normals_alg->SetInputConnection(polydata->GetProducerPort());
    #else
      normals_alg->SetInputData(polydata);
    #endif

    normals_alg->ComputePointNormalsOn();
    normals_alg->SplittingOff();
    normals_alg->ConsistencyOff();

    normals_alg->Update();

    vtkSmartPointer<vtkFloatArray> vtk_normals_estimated = vtkSmartPointer<vtkFloatArray>::New();
    vtk_normals_estimated->SetNumberOfComponents(3);
    vtk_normals_estimated->SetName("Normals");
    vtk_normals_estimated = vtkFloatArray::SafeDownCast(normals_alg->GetOutput()->GetPointData()->GetNormals());

    normals_for_orientation=vtk_normal_tcoords_to_vector(vtk_normals_estimated);
  }



  //Cluster the normals of the points.
  cv::Mat samples(normals_for_orientation.size()/downsample, 3, CV_32F);
  for( int y = 0; y < samples.rows; y++ ){
    for( int x = 0; x < samples.cols; x++ ){
      samples.at<float>(y,x)=normals_for_orientation[y*downsample][x];
    }
  }



  int cluster_count = model->m_num_walls;
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

  for (size_t i = 0; i < model->m_num_walls; i++) {
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
      transformFilter->SetInputConnection(polydata->GetProducerPort());
  #else
      transformFilter->SetInputData(polydata);
  #endif


  transformFilter->SetTransform(trans);
  transformFilter->Update();

  vtkSmartPointer<vtkPolyData> wall_aligned;
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




  return transformFilter2->GetOutput();

}

void auto_fix_pose( vtkSmartPointer<vtkPolyData> polydata){

  //make a deep copy of the polydata and do all the transformation on that copy. Afterwards set the input polydata to that transformed mesh. This is done so that we avoid to do something like: polydata= transformation->GetOutput() because the transformation was computed on the polydata and this will incurr a circular filtering.
  vtkSmartPointer<vtkPolyData> mesh_temp = vtkSmartPointer<vtkPolyData>::New();
  mesh_temp->DeepCopy(polydata);


  //fix orientation
  //


  //fix at the center of the world



}


void  Visualizer::draw_sphere(double x, double y, double z, double r, double g, double b ){

  // // Create a sphere
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
    actor->GetProperty()->SetColor(r, g, b);


    renderer->AddActor(actor);

}


void Visualizer::draw_line (double* pos1, double* pos2){
  // vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
  // lineSource->SetPoint1(pos1);
  // lineSource->SetPoint2(pos2);
  // lineSource->Update();
  //
  // vtkSmartPointer<vtkPolyDataMapper> line_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  // line_mapper->SetInputConnection(lineSource->GetOutputPort());
  // // line_mapper->StaticOn();
  // vtkSmartPointer<vtkActor> line_actor = vtkSmartPointer<vtkActor>::New();
  // line_actor->SetMapper(line_mapper);
  // line_actor->GetProperty()->SetLineWidth(8);
  // //line_actor->GetProperty()->SetColor(0.0, 0.0, 1.0); //(R,G,B)
  // //line_actor->GetProperty()->SetColor(0.6, 0.1, 0.1); //(R,G,B)
  // line_actor->GetProperty()->SetColor(0.66, 0.23, 0.22); //(R,G,B)
  //
  // renderer->AddActor(line_actor);
}



void Visualizer::clearAll(){
  model->clear();
  renderer->RemoveAllViewProps();
}


// void  Visualizer::updateView_only_color_change(int reset_camera){
//   std::cout << "update view" << std::endl;
//   renderer->RemoveAllViewProps();
//
//   //update the wall with the new points (wrapped on unwrapped)
//   vtkSmartPointer<vtkPolyData> wall=model->mesh->get_vtk_mesh();
//
// }


void  Visualizer::updateView(UpdateType update_type){
  std::cout << "update view" << std::endl;
  // renderer->RemoveAllViewProps();
  //
  // //update the wall with the new points (wrapped on unwrapped)
  // vtkSmartPointer<vtkPolyData> wall=model->mesh->get_vtk_mesh();
  //
  //
  // // Visualize
  // vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  // mapper->SetInputConnection(wall->GetProducerPort());
  // mapper->Update();
  // vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  // actor->SetMapper(mapper);
  //
  //
  // if (model->mesh->has_texture()){
  //   std::cout << "updating view with an actor with texture" << '\n';
  //   if (this->ui->colorComboBox->currentText()=="RGB (bright)")
  //     actor->SetTexture(model->mesh->get_texture_bright());
  //
  //   if (this->ui->colorComboBox->currentText()=="RGB (original)")
  //     actor->SetTexture(model->mesh->get_texture_original());
  //
  //   if (this->ui->colorComboBox->currentText()=="IR")
  //     actor->SetTexture(model->mesh->get_texture_ir());
  // }
  //
  // // actor->SetTexture(texture_bright);
  //
  //
  // // actor->GetProperty()->BackfaceCullingOn();
  //
  //
  // std::cout << "adding actor" << std::endl;
  // renderer->AddActor(actor);
  // if (reset_camera==1){
  //   set_camera_default_pos();
  //   renderer->ResetCamera();
  // }
  //
  // draw_sphere(0,0,0);
  //
  //
  //
  //
  // // draw_text_grid();
  // std::cout << "START UPDATE VIEW- wall" << std::endl;
  // // wall->Print(std::cout);
  // std::cout << "FIN: UPDATE VIEW- wall" << std::endl;
  //
  //
  // std::cout << "START UPDATE VIEW- actor" << std::endl;
  // // actor->Print(std::cout);
  // std::cout << "FIN: UPDATE VIEW- actor" << std::endl;
  //
  // std::cout << "updateing grid view" << std::endl;
  //
  // update_grid_view();
  // std::cout << "--------------rendering" << std::endl;
  // // renderer->Print(std::cout);
  // this->ui->qvtkWidget->GetRenderWindow()->Render();
  // std::cout << "--------------finished updateview" << std::endl;

  update_type=FULL_WALL;
  wall->Print(std::cout);


  switch (update_type){
    case FULL_WALL:{
      std::cout << "update view full wall" << '\n';
      vtkSmartPointer<vtkPolyData> wall=model->mesh->get_vtk_mesh();
      m_mapper_wall->SetInputConnection(wall->GetProducerPort());
      if (model->mesh->has_texture()){
        std::cout << "updating view with an actor with texture" << '\n';
        if (this->ui->colorComboBox->currentText()=="RGB (bright)")
          m_actor_wall->SetTexture(model->mesh->get_texture_bright());

        if (this->ui->colorComboBox->currentText()=="RGB (original)")
          m_actor_wall->SetTexture(model->mesh->get_texture_original());

        if (this->ui->colorComboBox->currentText()=="IR")
          m_actor_wall->SetTexture(model->mesh->get_texture_ir());
      }
      set_camera_default_pos();
      renderer->ResetCamera();
      this->ui->qvtkWidget->GetRenderWindow()->Render();
    }
    break;

    case ONLY_COLOR:{
      std::cout << "update view only color" << '\n';
      std::string color_combo_box_text= ui->colorComboBox->currentText().toStdString();
      if (model->mesh->has_texture() &&   ( color_combo_box_text=="RGB (bright)" || color_combo_box_text=="RGB (original)" || color_combo_box_text=="IR"   )  ){
        std::cout << "updating view with an actor with texture" << '\n';
        if (this->ui->colorComboBox->currentText()=="RGB (bright)")
          m_actor_wall->SetTexture(model->mesh->get_texture_bright());

        if (this->ui->colorComboBox->currentText()=="RGB (original)")
          m_actor_wall->SetTexture(model->mesh->get_texture_original());

        if (this->ui->colorComboBox->currentText()=="IR")
          m_actor_wall->SetTexture(model->mesh->get_texture_ir());
      }else{
        vtkSmartPointer<vtkPolyData> wall=model->mesh->get_vtk_mesh_only_color();
        m_mapper_wall->SetInputConnection(wall->GetProducerPort());
      }
    }
    break;

    case ONLY_CAMERA:
      std::cout << "update view only camera" << '\n'; //Dont nee to actually do anything because it will still call the render
    break;

    case ONLY_GRID:
      update_grid_view();
      set_camera_default_pos();
      renderer->ResetCamera();
    break;


  }
  this->ui->qvtkWidget->GetRenderWindow()->Render();

  // //SEcond way of doing it, a bit more efficient
  // vtkSmartPointer<vtkPolyData> wall=model->mesh->get_vtk_mesh();



}

void Visualizer::set_camera_default_pos(){
  std::cout << "camera to default" << std::endl;

  // renderer->GetActiveCamera()->SetPosition (0,0,1);
  // renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  // // renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
  // // renderer->GetActiveCamera()->Elevation(270);
  // renderer->GetActiveCamera()->OrthogonalizeViewUp();
  // renderer->ResetCamera();


  renderer->GetActiveCamera()->SetPosition (0,5,0);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetViewUp(0, 0, -1);
  // renderer->GetActiveCamera()->Elevation(270);
  // renderer->GetActiveCamera()->OrthogonalizeViewUp();
  renderer->ResetCamera();
}


void Visualizer::on_clearButton_clicked(){
  std::cout << "clearing the view" << std::endl;
  renderer->RemoveAllViewProps();
  this->ui->qvtkWidget->GetRenderWindow()->Render();
}

void Visualizer::on_unwrapButton_clicked(){
  std::cout << "unwrapping" << std::endl;
  model->mesh->set_state_unwrapped( !model->mesh->get_state_unwrapped() );

  if (model->mesh->has_data() && !model->mesh->has_unwrapped_data() ){


    if ( model->mesh->get_state_unwrapped()){
      model->mesh->compute_unwrap();
    }
    // model->write_points_to_mesh();  //Need to add the computed unwraps to the mesh so that the grid is correct

  }else if (model->mesh->has_data() && model->mesh->has_unwrapped_data()){

    //If the num of walls change we need to calculate a new unwrap
    if ( model->mesh->get_state_unwrapped() && model->mesh->get_nr_walls()!=model->mesh->get_nr_walls_baked()){
      model->mesh->compute_unwrap();
    }
  }
  // model->compute_unwrap(); //for cylinder chimneys

  model->create_grid();
  updateView();

}

void Visualizer::on_colorComboBox_currentIndexChanged(const QString & text){
  std::cout << text.toStdString() << std::endl;

  if (text.toStdString() == "Plain"){
    std::cout << "calculating plain colors" << std::endl;
    model->mesh->compute_plain_colors();
    model->mesh->set_state_selected_ir(false);
  }
  if (text.toStdString() == "RGB (bright)"){
    std::cout << "calculating RGB brightened colors" << std::endl;
    if (model->mesh->is_ply()){
      model->mesh->compute_bright_colors();
    }
    model->mesh->set_state_selected_ir(false);
  }
  if (text.toStdString() == "RGB (original)"){
    std::cout << "calculating RGB original colors" << std::endl;
    if (model->mesh->is_ply()){
      model->mesh->compute_original_colors();
    }
    model->mesh->set_state_selected_ir(false);
  }
  if (text.toStdString() == "IR"){
    std::cout << "calculating IR colors" << std::endl;
    if (!model->mesh->has_ir() ){
      select_ir_mesh();
    }
    model->mesh->set_state_has_ir(true);
    model->mesh->set_state_selected_ir(true);
  }
  if (text.toStdString() == "Depth"){
    std::cout << "calculating Depth colors" << std::endl;
    model->mesh->compute_depth_colors();
    model->mesh->set_state_selected_ir(false);
  }
  if (text.toStdString() == "Depth (colored)"){
    std::cout << "calculating Depth colors rgb" << std::endl;
    model->mesh->compute_depth_rgb_colors();
    model->mesh->set_state_selected_ir(false);
  }
  if (text.toStdString() == "Depth (defects)"){
    std::cout << "calculating Depth_defects colors" << std::endl;
    model->mesh->compute_depth_defects_colors();
    model->mesh->set_state_selected_ir(false);
  }
  if (text.toStdString() == "Curvature"){
    std::cout << "calculating Curvature colors" << std::endl;
    model->mesh->set_state_selected_ir(false);
  }

  updateView(ONLY_COLOR);

}

void Visualizer::select_ir_mesh(){
  std::cout << "selecting ir mesh" << std::endl;

  QString file_name;
  QString selfilter = tr("Mesh (*.obj *.ply)");
  file_name = QFileDialog::getOpenFileName(this,
             tr("Open File"), "./", selfilter);

  if (file_name.isEmpty()){
    return;
  }

  //Read that mesh like an obj normally- get the texture and the texture coordinates for it.
  // QString file_name="/media/alex/Data/Master/SHK/Data/Chimney/ir_test_1/researchDenslyTexturedMesh.obj";

  std::cout << "filename: " << file_name.toStdString() << std::endl;


  if (boost::ends_with(file_name.toStdString(), ".ply")) {
    std::cout << "reading .ply file" << std::endl;
    std::cout << "not implemented yet" << std::endl;
  }else if (boost::ends_with(file_name.toStdString(), ".obj")){
    std::cout << "reading .obj file" << std::endl;

    std::unique_ptr<OBJReader2> obj_reader(new OBJReader2());
    obj_reader->experimental_loading=model->m_experiemental_loading;
    // obj_reader->should_fix_orientation=model->m_fix_orientation;
    obj_reader->SetFileName(file_name.toStdString());
    obj_reader->Update();

    vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
    pngReader->SetFileName (obj_reader->GetTexturePath().data() );
    pngReader->Update();

    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
    texture->SetInputConnection(pngReader->GetOutputPort());

    // model->set_mesh(obj_reader->GetOutput());
    // model->set_texture(texture);

    //Dont set the mesh or texture, insted read the texture coordinate and the texture will be set to ir texture
     model->set_ir_texture(texture);


     //Get the texture coordinates and set them in the model
     //GEt texture coordinates
     vtkSmartPointer<vtkDataArray> tcoords = obj_reader->GetOutput()->GetPointData()->GetTCoords();
     if(tcoords){
       std::cout << "IR:the polydata has tcoords" << std::endl;
     }else{
       std::cout << "IR:the polydata does not have tcoords" << std::endl;
     }
     model->tcoords_ir=tcoords;

    //  m_tcoords_wrapped=vtk_normal_tcoords_to_vector(vtk_tcoords);



  }else{
    std::cout << "NOT VALID FORMAT" << std::endl;
    return;
  }

}

void Visualizer::on_flatShadingCheckBox_clicked(){
  std::cout << "change shading" << std::endl;


  if (ui->flatShadingCheckBox->isChecked()){
    m_actor_wall->GetProperty()->SetInterpolationToFlat();
    m_actor_wall->GetProperty()->SetAmbient(1.0);
    m_actor_wall->GetProperty()->SetDiffuse(0.0);
    m_actor_wall->GetProperty()->SetSpecular(0.0);
  }else{
    m_actor_wall->GetProperty()->SetInterpolationToPhong();
    m_actor_wall->GetProperty()->SetAmbient(0.0);
    m_actor_wall->GetProperty()->SetDiffuse(1.0);
    m_actor_wall->GetProperty()->SetSpecular(0.0);
  }

  updateView(ONLY_CAMERA);

}

void Visualizer::on_highCapDepthColorSlider_valueChanged(){
  double val= ui->highCapDepthColorSlider->value();
  std::cout << "setting high cap to " << val << '\n';
  model->mesh->set_high_cap_depth_color(val);
}

void Visualizer::on_lowCapDepthColorSlider_valueChanged(){
  double val= ui->lowCapDepthColorSlider->value();
  std::cout << "setting low cap to " << val << '\n';
  model->mesh->set_low_cap_depth_color(val);
}



void Visualizer::on_perspectiveCheckBox_clicked(){
  std::cout << "change perspective" << std::endl;

  int current_val=renderer->GetActiveCamera()->GetParallelProjection();
  if (current_val==0)
    renderer->GetActiveCamera()->SetParallelProjection(1);
  else
    renderer->GetActiveCamera()->SetParallelProjection(0);


  updateView(ONLY_CAMERA);

}

void Visualizer::on_selectButton_clicked(){
  std::cout << "selecting defects" << std::endl;
  model->m_selecting_defects=!model->m_selecting_defects;
}



void Visualizer::slotExit()
{
  qApp->exit();
}


void Visualizer::on_showGridInactiveCheckBox_clicked(){
  if (ui->showGridInactiveCheckBox->isChecked()){
    std::cout << "showing inactive grid cells" << std::endl;
    model->m_draw_grid_inactive=true;
  }else{
    std::cout << "hiding inactive grid cells" << std::endl;
    model->m_draw_grid_inactive=false;
  }
  update_grid_view();
}
void Visualizer::on_showGridActiveCheckBox_clicked(){
  if (ui->showGridActiveCheckBox->isChecked()){
    std::cout << "showing active grid cells" << std::endl;
    model->m_draw_grid_active=true;
  }else{
    std::cout << "hiding active grid cells" << std::endl;
    model->m_draw_grid_active=false;
  }
  update_grid_view();
}

void Visualizer::on_renderToImgButton_clicked(){
  std::cout << "rendering to img" << std::endl;
  std::string path= "/home/alex/Pictures/Renders/";
  std::string file_name = "render.png";

  path=path+file_name;



  //TEST:Get position, focal points and direction
  double* pos;
  double* focal_point;
  double* view_up;

  pos=renderer->GetActiveCamera()->GetPosition();
  focal_point=renderer->GetActiveCamera()->GetFocalPoint();
  view_up=renderer->GetActiveCamera()->GetViewUp();
  std::cout << "when starting " << std::endl;
  std::cout << "positon: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
  std::cout << "focal point: " << focal_point[0] << " " << focal_point[1] << " " << focal_point[2] << std::endl;
  std::cout << "view_up: " << view_up[0] << " " << view_up[1] << " " << view_up[2] << std::endl;




  renderer->ResetCamera();
  renderer->GetActiveCamera()->SetParallelProjection(1);
  renderer->GetActiveCamera()->Elevation(270);
  renderer->GetActiveCamera()->Roll(180);
  renderer->SetBackground(0.0,0.0,0.0);
  renderer->SetBackground2(0.0,0.0,0.0);



  this->ui->qvtkWidget->GetRenderWindow()->Render();


  // renderer->GetActiveCamera()->SetParallelProjection(0);


  vtkSmartPointer<vtkRenderLargeImage> renderLarge = vtkSmartPointer<vtkRenderLargeImage>::New();
  renderLarge->SetInput(renderer);
  renderLarge->SetMagnification(MAGNIFICATION);



  std::cout << "Saving image in " << path << std::endl;
  vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(path.data());
  writer->SetInputConnection(renderLarge->GetOutputPort());
  writer->Write();



  pos=renderer->GetActiveCamera()->GetPosition();
  focal_point=renderer->GetActiveCamera()->GetFocalPoint();
  view_up=renderer->GetActiveCamera()->GetViewUp();
  std::cout << "after setting the camera " << std::endl;
  std::cout << "positon: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
  std::cout << "focal point: " << focal_point[0] << " " << focal_point[1] << " " << focal_point[2] << std::endl;
  std::cout << "view_up: " << view_up[0] << " " << view_up[1] << " " << view_up[2] << std::endl;



  // renderer->ResetCamera();
  // renderer->GetActiveCamera()->SetParallelProjection(0);
  renderer->GetActiveCamera()->Elevation(-270);
  renderer->GetActiveCamera()->Roll(-180);
  renderer->SetBackground(1.0,1.0,1.0);
  renderer->SetBackground2(0.1,0.1,0.1);
  // renderer->ResetCamera();
  this->ui->qvtkWidget->GetRenderWindow()->Render();

  std::cout << "finished writing full img" << std::endl;


}

void Visualizer::on_renderGridCellButton_clicked(){
  std::cout << "getting grid cell" << std::endl;

  std::string path= "/home/alex/Pictures/Renders/";
  std::string file_name = "render.png";
  std::string path_img=path+file_name;

  if (model->m_points_wrapped.empty()){
    return;
  }


  //Unwrap if it¡s not unwrapped
  model->m_is_unwrapped=true;
  if (model->m_points_unwrapped.empty() && !model->m_points_wrapped.empty()){
    // model->compute_unwrap(); //for cylinder chimneys
    model->compute_unwrap2();
    // model->write_points_to_mesh();  //Need to add the computed unwraps to the mesh so that the grid is correct
    model->create_grid();
  }
  updateView();



  //Set camera
  renderer->GetActiveCamera()->SetPosition (0,0,1);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
  renderer->ResetCamera();
  renderer->GetActiveCamera()->SetParallelProjection(1);
  renderer->GetActiveCamera()->Elevation(270);
  // renderer->GetActiveCamera()->Pitch(270);
  renderer->GetActiveCamera()->Roll(180);
  renderer->SetBackground(0.0,0.0,0.0);
  renderer->SetBackground2(0.0,0.0,0.0);
  this->ui->qvtkWidget->GetRenderWindow()->Render();



  //Do stuff

  //Big img
  int full_img_magnification=6;
  render_to_file(path_img,full_img_magnification);

  //Cells
  for (size_t i = 0; i < model->m_grid.size(); i++) {
    renderer->ResetCamera(model->m_grid[i].data());
    std::string path_cell= path + "grid_cell_" + std::to_string(i) + ".png";
    int cell_magnification=3;
    render_to_file(path_cell,cell_magnification);

    //Read and crop
    cv::Mat img =cv::imread(path_cell);

    row_type corner_upper_left(3);
    row_type corner_lower_right(3);
    double pos_display_upper_left[3];
    double pos_display_lower_right[3];

    corner_upper_left[0]=model->m_grid[i][0];
    corner_upper_left[1]=model->m_grid[i][3];
    corner_upper_left[2]=model->m_grid[i][5];

    corner_lower_right[0]=model->m_grid[i][1];
    corner_lower_right[1]=model->m_grid[i][3];
    corner_lower_right[2]=model->m_grid[i][4];

    vtkInteractorObserver::ComputeWorldToDisplay (renderer, corner_upper_left[0],
                                                            corner_upper_left[1],
                                                            corner_upper_left[2],
                                                            pos_display_upper_left);

    vtkInteractorObserver::ComputeWorldToDisplay (renderer, corner_lower_right[0],
                                                            corner_lower_right[1],
                                                            corner_lower_right[2],
                                                            pos_display_lower_right);

    //Flip te position in display because Opencv has y origin at the upper left
    int* w_size;
    w_size = this->ui->qvtkWidget->GetRenderWindow()->GetSize();
    pos_display_lower_right[1]=w_size[1]-pos_display_lower_right[1];
    pos_display_upper_left[1]=w_size[1]-pos_display_upper_left[1];

    double size=fabs (pos_display_lower_right[0]*cell_magnification - pos_display_upper_left[0]*cell_magnification);

    cv::Mat grid_cell;
    cv::Rect rect(pos_display_upper_left[0]*cell_magnification,
                  pos_display_upper_left[1]*cell_magnification,
                  size,
                  size);
    grid_cell=img(rect);

    cv::imwrite( path_cell, grid_cell );

  }



  //Put the camera back
  renderer->GetActiveCamera()->SetPosition (0,0,1);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
  renderer->ResetCamera();


  // //Write big image
  // render_to_file(path_img,15);
  //
  //
  //
  //
  //
  // cv::Mat img =cv::imread(path_img);
  //
  //
  // for (size_t i = 0; i < model->m_grid.size(); i++) {
  //   row_type corner_upper_left(3);
  //   row_type corner_lower_right(3);
  //   double pos_display_upper_left[3];
  //   double pos_display_lower_right[3];
  //
  //   corner_upper_left[0]=model->m_grid[i][0];
  //   corner_upper_left[1]=model->m_grid[i][3];
  //   corner_upper_left[2]=model->m_grid[i][5];
  //
  //   corner_lower_right[0]=model->m_grid[i][1];
  //   corner_lower_right[1]=model->m_grid[i][3];
  //   corner_lower_right[2]=model->m_grid[i][4];
  //
  //
  //   draw_cell(model->m_grid[i],1,0,0);
  //   draw_sphere(corner_upper_left[0],corner_upper_left[1],corner_upper_left[2]);
  //   draw_sphere(corner_lower_right[0],corner_lower_right[1],corner_lower_right[2]);
  //
  //   vtkInteractorObserver::ComputeWorldToDisplay (renderer, corner_upper_left[0],
  //                                                           corner_upper_left[1],
  //                                                           corner_upper_left[2],
  //                                                           pos_display_upper_left);
  //
  //   vtkInteractorObserver::ComputeWorldToDisplay (renderer, corner_lower_right[0],
  //                                                           corner_lower_right[1],
  //                                                           corner_lower_right[2],
  //                                                           pos_display_lower_right);
  //
  //
  //
  //
  //
  //   //Flip te position in display because Opencv has y origin at the upper left
  //   int* w_size;
  //   w_size = this->ui->qvtkWidget->GetRenderWindow()->GetSize();
  //   pos_display_lower_right[1]=w_size[1]-pos_display_lower_right[1];
  //   pos_display_upper_left[1]=w_size[1]-pos_display_upper_left[1];
  //
  //
  //   std::cout << "pos_display_upper_left: " << pos_display_upper_left[0]*MAGNIFICATION << " " << pos_display_upper_left[1]*MAGNIFICATION << " " << pos_display_upper_left[2]*MAGNIFICATION << std::endl;
  //
  //   std::cout << "pos_display_lower_right: " << pos_display_lower_right[0]*MAGNIFICATION << " " << pos_display_lower_right[1]*MAGNIFICATION << " " << pos_display_lower_right[2]*MAGNIFICATION << std::endl;
  //
  //   double size=fabs (pos_display_lower_right[0]*MAGNIFICATION - pos_display_upper_left[0]*MAGNIFICATION);
  //
  //   cv::Mat grid_cell;
  //   cv::Rect rect(pos_display_upper_left[0]*MAGNIFICATION,
  //                 pos_display_upper_left[1]*MAGNIFICATION,
  //                 size,
  //                 size);
  //   grid_cell=img(rect);
  //
  //
  //   std::string path_cell= path + "grid_cell_" + std::to_string(i) + ".png";
  //   cv::imwrite( path_cell, grid_cell );
  //
  //   // break;
  //
  // }



  // vtkInteractorObserver::ComputeWorldToDisplay (renderer, corner[0], corner[1], corner[2], pos_display);

  // std::cout << "pos_display: " << pos_display[0] << " " << pos_display[1] << " " << pos_display[2] << std::endl;


  std::cout << "finished getting grid cells" << std::endl;
}


void Visualizer::on_renderWallsButton_clicked(){
  std::cout << "rendering walls" << std::endl;
  std::string path= "/home/alex/Pictures/Renders/";
  std::string file_name = "wall.png";
  std::string path_img=path+file_name;


  //Computer unwrap if it's not done
  if (model->m_points_unwrapped.empty() && !model->m_points_wrapped.empty()){
    model->compute_unwrap2();
    // model->write_points_to_mesh();
    model->create_grid();
  }

  //wrap it
  //Unwrap if it¡s not unwrapped
  model->m_is_unwrapped=false;
  updateView();

  for (size_t i = 0; i < model->planes.size(); i++) {
    int clust_idx= model->planes[i].index_cluster;
    double angle=0.0;

    //roate the mesh
    vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
    angle= interpolate (model->planes[i].angle, 0.0, 1.0,  -M_PI, M_PI);
    angle= angle * 180.0/M_PI;
    trans->RotateZ(angle);

    std::cout << "---------rotating angle: " << angle << std::endl;
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    #if VTK_MAJOR_VERSION <= 5
      transformFilter->SetInputConnection(model->m_wall->GetProducerPort());
    #else
      transformFilter->SetInputData(model->m_wall);
    #endif



    transformFilter->SetTransform(trans);
    transformFilter->Update();
    //Get the new points and the new normals
    model->m_wall=transformFilter->GetOutput();
    model->read_info();
    updateView();


    //Rotate the clustered cloud
    Eigen::Affine3f trans_cloud = Eigen::Affine3f::Identity();
    double r = interpolate (model->planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
    trans_cloud.rotate (Eigen::AngleAxisf (r, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*(model->clustered_clouds_original[clust_idx]),
                              *(model->clustered_clouds_original[clust_idx]), trans_cloud);



    //from the inliers of that cloud get the bounds
    double bounds[6];
    bounds[0]=999999;   //x_min
    bounds[1]=-999999;  //x_max
    bounds[2]=999999;   //y_min
    bounds[3]=-999999;  //y_max
    bounds[4]=999999;   //z_min
    bounds[5]=-999999;  //z_max
    for (size_t in_idx = 0; in_idx < model->m_inliers_vec[clust_idx]->indices.size (); in_idx++) {

      double x=model->clustered_clouds_original[clust_idx]->points[ model->m_inliers_vec[clust_idx]->indices[in_idx]].x;
      double y=model->clustered_clouds_original[clust_idx]->points[ model->m_inliers_vec[clust_idx]->indices[in_idx]].y;
      double z=model->clustered_clouds_original[clust_idx]->points[ model->m_inliers_vec[clust_idx]->indices[in_idx]].z;

      //x min
      if (x < bounds[0])
        bounds[0]=x;
      //X_max
      if (x > bounds[1])
        bounds[1]=x;
      //y min
      if (y < bounds[2])
        bounds[2]=y;
      //y_max
      if (y > bounds[3])
        bounds[3]=y;
      //z min
      if (z < bounds[4])
        bounds[4]=z;
      //z_max
      if (z > bounds[5])
        bounds[5]=z;

    }


    row_type bounds_vec(std::begin(bounds), std::end(bounds));
    draw_cell(bounds_vec, 1.0, 0.0, 0.0);

    //Set the camera
    renderer->GetActiveCamera()->SetPosition (0.0, 0.0, 0.0);
    renderer->GetActiveCamera()->SetViewUp (0.0, 0.0, 1.0);
    row_type focal_point(3);
    focal_point[0]= (bounds[0]+  bounds[1])/2.0;
    focal_point[1]= (bounds[2]+  bounds[3])/2.0;
    focal_point[2]= (bounds[4]+  bounds[5])/2.0;
    draw_sphere(focal_point[0],focal_point[1], focal_point[2]);
    renderer->GetActiveCamera()->SetFocalPoint (focal_point[0],focal_point[1], focal_point[2]);
    renderer->ResetCamera(bounds);


    //Render
    std::string path_wall= path + "wall_" + std::to_string(i) + ".png";
    render_to_file(path_wall,1);





    //Rotate them back
    vtkSmartPointer<vtkTransform> trans_back = vtkSmartPointer<vtkTransform>::New();
    angle= interpolate (model->planes[i].angle, 0.0, 1.0,  -M_PI, M_PI);
    angle= angle * 180.0/M_PI;
    trans_back->RotateZ(-angle);

    std::cout << "+rotating angle: back " << -angle << std::endl;
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter_back = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
      transformFilter->SetInputConnection(model->m_wall->GetProducerPort());
    #else
      transformFilter->SetInputData(model->m_wall);
    #endif
    transformFilter_back->SetTransform(trans_back);
    transformFilter_back->Update();
    //Get the new points and the new normals
    model->m_wall=transformFilter_back->GetOutput();
    model->read_info();
    updateView();



    Eigen::Affine3f trans_cloud_back = Eigen::Affine3f::Identity();
    double r_back = interpolate (model->planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
    trans_cloud_back.rotate (Eigen::AngleAxisf (-r_back, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*(model->clustered_clouds_original[clust_idx]),
                              *(model->clustered_clouds_original[clust_idx]), trans_cloud_back);



    //Put the camera back
    renderer->GetActiveCamera()->SetPosition (0,0,1);
    renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
    // renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
    renderer->GetActiveCamera()->Elevation(270);
    renderer->GetActiveCamera()->OrthogonalizeViewUp();
    renderer->ResetCamera();
    this->ui->qvtkWidget->GetRenderWindow()->Render();


  }

  // updateView();
  return;


  // renderer->GetActiveCamera()->SetPosition (0.0, 0.0, 0.0);
  // renderer->GetActiveCamera()->SetFocalPoint(model->m_plane_centers[4][0],
  //                                            model->m_plane_centers[4][1],
  //                                            model->m_plane_centers[4][2]);
  //
  //
  // renderer->GetActiveCamera()->SetViewUp(0, 0, 1);
  // // renderer->ResetCamera();
  // renderer->GetActiveCamera()->Dolly (0.5);

  // for (size_t i = 0; i < model->m_num_walls; i++) {
  //   renderer->GetActiveCamera()->SetPosition (model->m_plane_centers[i][0],
  //                                             model->m_plane_centers[i][1],
  //                                             model->m_plane_centers[i][2]);
  //
  //   renderer->GetActiveCamera()->SetFocalPoint(0.0, 0.0, 0.0);
  //   renderer->GetActiveCamera()->SetViewUp(0, 0, 1);
  //   renderer->ResetCamera();
  //   // renderer->GetActiveCamera()->Dolly (1.5);
  //   renderer->GetActiveCamera()->Zoom (2.5);
  //
  //   this->ui->qvtkWidget->GetRenderWindow()->Render();
  //
  //
  //   //Write wall
  //   file_name= "wall_" + std::to_string(i) + ".png";
  //   path_img= path + file_name;
  //
  //   vtkSmartPointer<vtkRenderLargeImage> renderLarge = vtkSmartPointer<vtkRenderLargeImage>::New();
  //   renderLarge->SetInput(renderer);
  //   renderLarge->SetMagnification(MAGNIFICATION);
  //   std::cout << "Saving image in " << path_img << std::endl;
  //   vtkSmartPointer<vtkPNGWriter> writer =
  //     vtkSmartPointer<vtkPNGWriter>::New();
  //   writer->SetFileName(path_img.data());
  //   writer->SetInputConnection(renderLarge->GetOutputPort());
  //   writer->Write();
  // }


  model->wrap_grid();

  for (size_t i = 0; i < model->m_grid_wrapped.size(); i++) {
    // draw_sphere(model->m_grid_wrapped[i][0][0], model->m_grid_wrapped[i][0][1], model->m_grid_wrapped[i][0][2]);
    // draw_sphere(model->m_grid_wrapped[i][1][0], model->m_grid_wrapped[i][1][1], model->m_grid_wrapped[i][1][2]);
    // draw_sphere(model->m_grid_wrapped[i][2][0], model->m_grid_wrapped[i][2][1], model->m_grid_wrapped[i][2][2]);
    // draw_sphere(model->m_grid_wrapped[i][3][0], model->m_grid_wrapped[i][3][1], model->m_grid_wrapped[i][3][2]);
  }

  //middle point
  for (size_t i = 0; i < model->m_grid_wrapped.size(); i++) {
    row_type mid_point(3, 0.0);

    mid_point[0]= (model->m_grid_wrapped[i][0][0] + model->m_grid_wrapped[i][2][0] )/2.0;
    mid_point[1]= (model->m_grid_wrapped[i][0][1] + model->m_grid_wrapped[i][2][1] )/2.0;
    mid_point[2]= (model->m_grid_wrapped[i][0][2] + model->m_grid_wrapped[i][2][2] )/2.0;

    //Draw the corner just in case
    draw_sphere(model->m_grid_wrapped[i][0][0],
                model->m_grid_wrapped[i][0][1],
                model->m_grid_wrapped[i][0][2], 1.0, 0.0, 0.0);
    draw_sphere(model->m_grid_wrapped[i][1][0],
                model->m_grid_wrapped[i][1][1],
                model->m_grid_wrapped[i][1][2], 0.0, 1.0, 0.0);
    draw_sphere(model->m_grid_wrapped[i][2][0],
                model->m_grid_wrapped[i][2][1],
                model->m_grid_wrapped[i][2][2], 0.0, 0.0, 1.0);
    draw_sphere(model->m_grid_wrapped[i][3][0],
                model->m_grid_wrapped[i][3][1],
                model->m_grid_wrapped[i][3][2], 0.0, 0.0, 0.0);

    draw_sphere(mid_point[0], mid_point[1], mid_point[2], 0.0, 0.0, 1.0);

    row_type normal(3, 0.0);
    //To get  normal going inside the chimney we need a vector from point 1->0 and from points 1->2. Cross product
    //therefore we do 0-1 and 2-1

    //To get  normal going inside the chimney we need a vector from point min->0 and from points min->1. Cross product
    //therefore we do 0-mid and 1-mid

    Eigen::Vector3f vec1( model->m_grid_wrapped[i][1][0] - mid_point[0],
                          model->m_grid_wrapped[i][1][1] - mid_point[1],
                          model->m_grid_wrapped[i][1][2] - mid_point[2]);

    Eigen::Vector3f vec2( model->m_grid_wrapped[i][2][0] - mid_point[0],
                          model->m_grid_wrapped[i][2][1] - mid_point[1],
                          model->m_grid_wrapped[i][2][2] - mid_point[2]);

    Eigen::Vector3f cross = vec2.cross(vec1);
    double cross_vec[3];
    cross_vec[0]=cross[0];
    cross_vec[1]=cross[1];
    cross_vec[2]=cross[2];

    std::cout << "cross product is " << cross[0] << " " << cross[1] << " " << cross[2] << std::endl;

    //
    // draw_sphere(vec1[0],
    //             vec1[1],
    //             vec1[2] , 0.0, 1.0, 0.0);
    //
    // draw_sphere(vec2[0],
    //             vec2[1],
    //             vec2[2] , 0.0, 1.0, 1.0);

    draw_sphere(cross_vec[0],
                cross_vec[1],
                cross_vec[2] , 1.0, 0.0, 0.0);


    //Make a plane with center the mid point and normal the cross vector. Push it some value. The new center will be the camera
    // vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    // planeSource->SetCenter(mid_point[0], mid_point[0], mid_point[0]);
    // planeSource->SetNormal( planes[i].coef.values[0],
    //                         planes[i].coef.values[1],
    //                         planes[i].coef.values[2]);
    // planeSource->Push(-planes[i].coef.values[3]);
    // planeSource->Update();

    row_type pos_camera(3);

    int dist=10;
    pos_camera[0]= mid_point[0] + dist*cross[0];
    pos_camera[1]= mid_point[1] + dist*cross[1];
    pos_camera[2]= mid_point[2] + dist*cross[2];

    // draw_sphere(pos_camera[0],pos_camera[1],pos_camera[2], 0.0, 0.0, 1.0 );

    renderer->GetActiveCamera()->SetPosition (pos_camera[0],
                                              pos_camera[1],
                                              pos_camera[2]);

    renderer->GetActiveCamera()->SetFocalPoint(mid_point[0], mid_point[1], mid_point[2]);
    renderer->GetActiveCamera()->SetViewUp(0, 0, 1);


    // this->ui->qvtkWidget->GetRenderWindow()->Render();
    // this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->Render();
    // this->ui->qvtkWidget->GetRenderWindow()->Render();
    // this->ui->qvtkWidget->update();
    // this->ui->qvtkWidget->updateGeometry();
    // renderer->Render();
    double bounds[6]= {0};;
    calculate_bounds(model->m_grid_wrapped[i], bounds);
    std::cout << "outside funct: " << std::endl;
    std::cout  << "xmin: " << bounds[0] << " "
               << "xmax: " << bounds[1] << std::endl
               << "ymin: " << bounds[2] << " "
               << "ymax: " << bounds[3] << std::endl
               << "zmin: " << bounds[4] << " "
               << "zmax: " << bounds[5] << std::endl;
    renderer->ResetCamera(bounds);

    // this->ui->qvtkWidget->render();

    render_to_file(path_img);

    //Read the image with opencv
    //Project the coords of the corner into display
    //In display coordinate get the maximum and minimum  and y
    //Create a rectangle with that and crop it


    // SetCameraPositionOrientation( renderer->GetActiveCamera(), mid_point.data(), cross_vec );

    // break;
  }

  //For each one of our grid, calculate its normal
  //Put a camera in front of it
  //Render a big image
  //Get the points in display coordinates
  //Crop the image


  this->ui->qvtkWidget->GetRenderWindow()->Render();
  std::cout << "finished rendering walls" << std::endl;
}


void Visualizer::on_renderGridWrappedButton_clicked (){
  std::cout << "writing the grid wrapped" << std::endl;

  std::string path= "/home/alex/Pictures/Renders/";
  std::string file_name = "wall.png";
  std::string path_img=path+file_name;

  //If it hasn't been unwrapped, unwrap it


  model->wrap_grid();

  for (size_t i = 0; i < model->m_grid_wrapped.size(); i++) {
    // draw_sphere(model->m_grid_wrapped[i][0][0], model->m_grid_wrapped[i][0][1], model->m_grid_wrapped[i][0][2]);
    // draw_sphere(model->m_grid_wrapped[i][1][0], model->m_grid_wrapped[i][1][1], model->m_grid_wrapped[i][1][2]);
    // draw_sphere(model->m_grid_wrapped[i][2][0], model->m_grid_wrapped[i][2][1], model->m_grid_wrapped[i][2][2]);
    // draw_sphere(model->m_grid_wrapped[i][3][0], model->m_grid_wrapped[i][3][1], model->m_grid_wrapped[i][3][2]);
  }

  //middle point
  for (size_t i = 0; i < model->m_grid_wrapped.size(); i++) {
    row_type mid_point(3, 0.0);

    mid_point[0]= (model->m_grid_wrapped[i][0][0] + model->m_grid_wrapped[i][2][0] )/2.0;
    mid_point[1]= (model->m_grid_wrapped[i][0][1] + model->m_grid_wrapped[i][2][1] )/2.0;
    mid_point[2]= (model->m_grid_wrapped[i][0][2] + model->m_grid_wrapped[i][2][2] )/2.0;

    //Draw the corner just in case
    // draw_sphere(model->m_grid_wrapped[i][0][0],
    //             model->m_grid_wrapped[i][0][1],
    //             model->m_grid_wrapped[i][0][2], 1.0, 0.0, 0.0);
    // draw_sphere(model->m_grid_wrapped[i][1][0],
    //             model->m_grid_wrapped[i][1][1],
    //             model->m_grid_wrapped[i][1][2], 0.0, 1.0, 0.0);
    // draw_sphere(model->m_grid_wrapped[i][2][0],
    //             model->m_grid_wrapped[i][2][1],
    //             model->m_grid_wrapped[i][2][2], 0.0, 0.0, 1.0);
    // draw_sphere(model->m_grid_wrapped[i][3][0],
    //             model->m_grid_wrapped[i][3][1],
    //             model->m_grid_wrapped[i][3][2], 0.0, 0.0, 0.0);
    //
    // draw_sphere(mid_point[0], mid_point[1], mid_point[2], 0.0, 0.0, 1.0);


    //Draw the corner but with lines;
    // if (i==16){
      draw_line(model->m_grid_wrapped[i][0].data(), model->m_grid_wrapped[i][1].data());
      draw_line(model->m_grid_wrapped[i][1].data(), model->m_grid_wrapped[i][2].data());
      draw_line(model->m_grid_wrapped[i][2].data(), model->m_grid_wrapped[i][3].data());
      draw_line(model->m_grid_wrapped[i][3].data(), model->m_grid_wrapped[i][0].data());
    // }



    row_type normal(3, 0.0);
    //To get  normal going inside the chimney we need a vector from point 1->0 and from points 1->2. Cross product
    //therefore we do 0-1 and 2-1

    //To get  normal going inside the chimney we need a vector from point min->0 and from points min->1. Cross product
    //therefore we do 0-mid and 1-mid

    Eigen::Vector3f vec1( model->m_grid_wrapped[i][1][0] - mid_point[0],
                          model->m_grid_wrapped[i][1][1] - mid_point[1],
                          model->m_grid_wrapped[i][1][2] - mid_point[2]);

    Eigen::Vector3f vec2( model->m_grid_wrapped[i][2][0] - mid_point[0],
                          model->m_grid_wrapped[i][2][1] - mid_point[1],
                          model->m_grid_wrapped[i][2][2] - mid_point[2]);

    Eigen::Vector3f cross = vec2.cross(vec1);
    double cross_vec[3];
    cross_vec[0]=cross[0];
    cross_vec[1]=cross[1];
    cross_vec[2]=cross[2];

    std::cout << "cross product is " << cross[0] << " " << cross[1] << " " << cross[2] << std::endl;

    //
    // draw_sphere(vec1[0],
    //             vec1[1],
    //             vec1[2] , 0.0, 1.0, 0.0);
    //
    // draw_sphere(vec2[0],
    //             vec2[1],
    //             vec2[2] , 0.0, 1.0, 1.0);

    // draw_sphere(cross_vec[0],
    //             cross_vec[1],
    //             cross_vec[2] , 1.0, 0.0, 0.0);


    //Make a plane with center the mid point and normal the cross vector. Push it some value. The new center will be the camera
    // vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    // planeSource->SetCenter(mid_point[0], mid_point[0], mid_point[0]);
    // planeSource->SetNormal( planes[i].coef.values[0],
    //                         planes[i].coef.values[1],
    //                         planes[i].coef.values[2]);
    // planeSource->Push(-planes[i].coef.values[3]);
    // planeSource->Update();

    row_type pos_camera(3);

    int dist=10;
    pos_camera[0]= mid_point[0] + dist*cross[0];
    pos_camera[1]= mid_point[1] + dist*cross[1];
    pos_camera[2]= mid_point[2] + dist*cross[2];

    // draw_sphere(pos_camera[0],pos_camera[1],pos_camera[2], 0.0, 0.0, 1.0 );

    renderer->GetActiveCamera()->SetPosition (pos_camera[0],
                                              pos_camera[1],
                                              pos_camera[2]);

    renderer->GetActiveCamera()->SetFocalPoint(mid_point[0], mid_point[1], mid_point[2]);
    renderer->GetActiveCamera()->SetViewUp(0, 0, 1);


    // this->ui->qvtkWidget->GetRenderWindow()->Render();
    // this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->Render();
    // this->ui->qvtkWidget->GetRenderWindow()->Render();
    // this->ui->qvtkWidget->update();
    // this->ui->qvtkWidget->updateGeometry();
    // renderer->Render();
    double bounds[6]= {0};;
    calculate_bounds(model->m_grid_wrapped[i], bounds);
    std::cout << "outside funct: " << std::endl;
    std::cout  << "xmin: " << bounds[0] << " "
               << "xmax: " << bounds[1] << std::endl
               << "ymin: " << bounds[2] << " "
               << "ymax: " << bounds[3] << std::endl
               << "zmin: " << bounds[4] << " "
               << "zmax: " << bounds[5] << std::endl;
    renderer->ResetCamera(bounds);

    // this->ui->qvtkWidget->render();

    file_name= "grid_w_" + std::to_string(i) + ".png";
    path_img= path + file_name;

    render_to_file(path_img,2);

    //Read the image with opencv
    //Project the coords of the corner into display
    //In display coordinate get the maximum and minimum  and y
    //Create a rectangle with that and crop it


    // SetCameraPositionOrientation( renderer->GetActiveCamera(), mid_point.data(), cross_vec );

    // break;
  }

  //For each one of our grid, calculate its normal
  //Put a camera in front of it
  //Render a big image
  //Get the points in display coordinates
  //Crop the image


  this->ui->qvtkWidget->GetRenderWindow()->Render();
  std::cout << "finished rendering walls" << std::endl;
}



//corners contains points. We need to calculate the min and max in each dimension
void Visualizer::calculate_bounds( matrix_type corners, double* bounds){
  // double bounds[6];
  // double *bounds = (double*)malloc(6*sizeof(double));
  bounds[0]=std::numeric_limits<double>::max();     //x_min
  bounds[1]=std::numeric_limits<double>::lowest();  //x_max
  bounds[2]=std::numeric_limits<double>::max();     //y_min
  bounds[3]=std::numeric_limits<double>::lowest();  //y_max
  bounds[4]=std::numeric_limits<double>::max();     //z_min
  bounds[5]=std::numeric_limits<double>::lowest();  //z_max

  for (size_t i = 0; i < corners.size(); i++) {
    //x min
    if (corners[i][0] < bounds[0])
      bounds[0]=corners[i][0];
    //X_max
    if (corners[i][0] > bounds[1])
      bounds[1]=corners[i][0];

    //y min
    if (corners[i][1] < bounds[2])
      bounds[2]=corners[i][1];
    //y_max
    if (corners[i][1] > bounds[3])
      bounds[3]=corners[i][1];


    //z min
    if (corners[i][2] < bounds[4])
      bounds[4]=corners[i][2];
    //z_max
    if (corners[i][2] > bounds[5])
      bounds[5]=corners[i][2];
  }

  // std::cout << "inside funct: " << std::endl;
  std::cout  << "xmin: " << bounds[0] << " "
             << "xmax: " << bounds[1] << std::endl
             << "ymin: " << bounds[2] << " "
             << "ymax: " << bounds[3] << std::endl
             << "zmin: " << bounds[4] << " "
             << "zmax: " << bounds[5] << std::endl;

}


void Visualizer::set_cam_pose( vtkCamera* cam, double position[3],
                                         double orientation[3] )
{
    double focus[3];
    double viewup[3];

    if( cam == NULL )
        return;

    focus[0] = position[0] - -cos(orientation[0])*sin(orientation[1]);
    focus[1] = position[1] - sin(orientation[0]);
    focus[2] = position[2] - cos(orientation[0])*cos(orientation[1]);

    viewup[0] = cos(orientation[1])*sin(orientation[2])+
                sin(orientation[1])*sin(orientation[2])*cos(orientation[2]);
    viewup[1] = cos(orientation[0])*cos(orientation[2]);
    viewup[2] = sin(orientation[1])*sin(orientation[2])-
                cos(orientation[1])*sin(orientation[0])*cos(orientation[2]);

    //set the camera position and orientation
    cam->SetPosition( position );
    // cam->SetViewUp( viewup );
    cam->SetViewUp( 0, 0 , 1 );
    cam->SetFocalPoint( focus );

}




void Visualizer::draw_grid(){
  //Need to do it by iterating twice so that the active ones are drawn on top (after) the inactive cells
  for (size_t i = 0; i < model->m_grid.size(); i++) {
    if(model->m_grid_cells_active[i]==0 && model->m_draw_grid_inactive){
      std::cout << "draw_grid::drawing inactive grid" << std::endl;
      draw_cell(model->m_grid[i], 0.5, 0.5, 0.5);
    }
  }

  for (size_t i = 0; i < model->m_grid.size(); i++) {
    if(model->m_grid_cells_active[i]==1 && model->m_draw_grid_active){
      std::cout << "draw_grid::drawing active grid" << std::endl;
      draw_cell(model->m_grid[i], 1.0, 0.0, 0.0);
    }
  }

}

void Visualizer::draw_cell(row_type bounds, double r, double g, double b){
  std::cout << "vis::draw cell" << std::endl;
  vtkSmartPointer<vtkOutlineSource> outlineSource = vtkSmartPointer<vtkOutlineSource>::New();

  std::cout << "vis::draw cell: setting bounds" << std::endl;
  outlineSource->SetBounds(bounds.data());
  std::cout << "vis::draw cell: updating" << std::endl;
  outlineSource->Update();

  std::cout << "vis::draw cell: creating outline" << std::endl;
  vtkSmartPointer<vtkPolyData> outline = outlineSource->GetOutput();

  // Create a mapper and actor
  std::cout << "vis::draw cell: create mapper" << std::endl;
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();


  std::cout << "vis::draw cell: setting connections" << std::endl;

  #if VTK_MAJOR_VERSION <= 5
    mapper->SetInputConnection(outline->GetProducerPort());
  #else
    mapper->SetInputData(outline);
  #endif



  std::cout << "vis::draw cell: creating actor" << std::endl;

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(r, g, b); //(R,G,B)
  // actor->GetProperty()->BackfaceCullingOn();

  std::cout << "vis::draw cell: adding actor to vector" << std::endl;

  m_grid_actors.push_back(actor);

  std::cout << "vis::draw cell: adding actor to renderer" << std::endl;

  renderer->AddActor(actor);

}

void Visualizer::draw_text_grid(){

  // if (model->m_grid.size()==0)
  //   return;
  //
  // renderer->RemoveActor(m_grid_metric_actor);
  //
  // int* w_size;
  // w_size = this->ui->qvtkWidget->GetRenderWindow()->GetSize();
  //
  // int active = std::count(model->m_grid_cells_active.begin(), model->m_grid_cells_active.end(), 1);
  // int total = model->m_grid.size();
  // std::string grid_metric = std::to_string (active) + "/" + std::to_string(total);
  // m_grid_metric_actor->SetInput ( grid_metric.data() );
  //
  // m_grid_metric_actor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
  // m_grid_metric_actor->GetPosition2Coordinate()->SetCoordinateSystemToNormalizedViewport();
  // m_grid_metric_actor->SetPosition(0.9,0.0);
  //
  //
  // m_grid_metric_actor->GetTextProperty()->SetFontSize ( 24 );
  // m_grid_metric_actor->GetTextProperty()->SetColor ( 0.1, 0.1, 0.1 );
  // m_grid_metric_actor->GetTextProperty()->ShadowOn ();
  // // textActor->GetTextProperty()->SetShadowOffset (int, int);
  // renderer->AddActor2D ( m_grid_metric_actor );
  //

}



void Visualizer::grid_changed_slot(){
  update_grid_view();
}

void Visualizer::update_grid_view(){
  std::cout << "vis::update_grid_view" << std::endl;
  for (size_t i = 0; i < m_grid_actors.size(); i++) {
    std::cout << "remove actor" << std::endl;
    renderer->RemoveActor(m_grid_actors[i]);
  }
  m_grid_actors.clear();

  std::cout << "Visualizer::update_grid_view:: drawing grid" << std::endl;
  draw_grid();
  // draw_text_grid();

  std::cout << "Visualizer::update_grid_view: RENDER" << std::endl;
  this->ui->qvtkWidget->GetRenderWindow()->Render();
}



void Visualizer::world_to_display(double* point_world, double* point_display){



  vtkInteractorObserver::ComputeWorldToDisplay (renderer, point_world[0],
                                                          point_world[1],
                                                          point_world[2],
                                                          point_display);

  //Flip te position in display because Opencv has y origin at the upper left
  int* w_size;
  w_size = this->ui->qvtkWidget->GetRenderWindow()->GetSize();
  point_display[1]=w_size[1]-point_display[1];
}




void Visualizer::render_to_file(std::string path, int magnification){
    vtkSmartPointer<vtkRenderLargeImage> renderLarge = vtkSmartPointer<vtkRenderLargeImage>::New();
    renderLarge->SetInput(renderer);
    renderLarge->SetMagnification(magnification);
    std::cout << "Saving image in " << path << std::endl;
    vtkSmartPointer<vtkPNGWriter> writer =
      vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(path.data());
    writer->SetInputConnection(renderLarge->GetOutputPort());
    writer->Write();

}


void Visualizer::render_full_img(){
  if (model->m_points_wrapped.empty()){
    return;
  }

  //If it's not unwraped, unwrap it
  model->m_is_unwrapped=true;
  if (model->m_points_unwrapped.empty() && !model->m_points_wrapped.empty()){
    model->compute_unwrap2();
    // model->write_points_to_mesh();  //Need to add the computed unwraps to the mesh so that the grid is correct
    model->create_grid();
  }
  updateView();


  //If the path does not exist, create it
  std::string path_sufix= "/Full_img";
  std::string path=model->m_path_global+ path_sufix;
  std::cout << "Path to write in is " << path << std::endl;
  if (!boost::filesystem::is_directory(path) ){
    // std::cout << "path does not exist, creating it" << std::endl;
    if (!boost::filesystem::create_directories(path)){
      std::cout << "Path could not be created or accessed" << std::endl;
      return;
    }else{
      // std::cout << "created the path correctly" << std::endl;
    }
  }

  std::string file_name= "/full_img.png";
  path=model->m_path_global+ path_sufix + file_name;

  //Set camera and background
  renderer->GetActiveCamera()->SetPosition (0,0,1);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
  renderer->ResetCamera();
  renderer->GetActiveCamera()->SetParallelProjection(1);
  renderer->GetActiveCamera()->Elevation(270);
  // renderer->GetActiveCamera()->Pitch(270);
  renderer->GetActiveCamera()->Roll(180);
  // renderer->SetBackground(0.0,0.0,0.0);
  // renderer->SetBackground2(0.0,0.0,0.0);
  this->ui->qvtkWidget->GetRenderWindow()->Render();

  //render
  render_to_file(path,model->m_magnification_full_img);

  //also render the grid
  model->m_draw_grid_active=true;
  for (size_t i = 0; i < model->m_grid.size(); i++) {
    model->m_grid_cells_active[i]=1;
  }
  update_grid_view();

  file_name= "/full_img_grid.png";
  path=model->m_path_global+ path_sufix + file_name;
  render_to_file(path,model->m_magnification_full_img);

  for (size_t i = 0; i < model->m_grid.size(); i++) {
    model->m_grid_cells_active[i]=0;
  }
  update_grid_view();


  //Set the background and camera back to default
  renderer->SetBackground(1.0,1.0,1.0);
  renderer->SetBackground2(0.1,0.1,0.1);
  renderer->GetActiveCamera()->SetParallelProjection(0);
  this->ui->qvtkWidget->GetRenderWindow()->Render();

}


void Visualizer::render_grid_unwrapped(){
  if (model->m_points_wrapped.empty()){
    return;
  }

  //If it's not unwraped, unwrap it
  model->m_is_unwrapped=true;
  if (model->m_points_unwrapped.empty() && !model->m_points_wrapped.empty()){
    model->compute_unwrap2();
    // model->write_points_to_mesh();  //Need to add the computed unwraps to the mesh so that the grid is correct
    model->create_grid();
  }
  updateView();


  //If the path does not exist, create it
  std::string path_sufix= "/Grid_unwrapped";
  std::string path=model->m_path_global+ path_sufix;
  std::cout << "Path to write in is " << path << std::endl;
  if (!boost::filesystem::is_directory(path) ){
    // std::cout << "path does not exist, creating it" << std::endl;
    if (!boost::filesystem::create_directories(path)){
      std::cout << "Path could not be created or accessed" << std::endl;
      return;
    }else{
      // std::cout << "created the path correctly" << std::endl;
    }
  }


  //Set camera
  renderer->GetActiveCamera()->SetPosition (0,0,1);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
  renderer->ResetCamera();
  renderer->GetActiveCamera()->SetParallelProjection(1);
  renderer->GetActiveCamera()->Elevation(270);
  // renderer->GetActiveCamera()->Pitch(270);
  renderer->GetActiveCamera()->Roll(180);
  // renderer->SetBackground(0.0,0.0,0.0);
  // renderer->SetBackground2(0.0,0.0,0.0);
  this->ui->qvtkWidget->GetRenderWindow()->Render();




  //Render each cell
  for (size_t i = 0; i < model->m_grid.size(); i++) {
    renderer->ResetCamera(model->m_grid[i].data());
    this->ui->qvtkWidget->GetRenderWindow()->Render();
    std::string path_cell= path + "/grid_cell_" + std::to_string(i) + ".png";
    render_to_file(path_cell,model->m_magnification_grid_unwrapped);

    //Read and crop
    cv::Mat img =cv::imread(path_cell);

    row_type corner_upper_left(3);
    row_type corner_lower_right(3);
    double pos_display_upper_left[3];
    double pos_display_lower_right[3];

    corner_upper_left[0]=model->m_grid[i][0];
    corner_upper_left[1]=model->m_grid[i][3];
    corner_upper_left[2]=model->m_grid[i][5];

    corner_lower_right[0]=model->m_grid[i][1];
    corner_lower_right[1]=model->m_grid[i][3];
    corner_lower_right[2]=model->m_grid[i][4];

    world_to_display(corner_upper_left.data(),pos_display_upper_left);
    world_to_display(corner_lower_right.data(),pos_display_lower_right);

    double size=fabs (pos_display_lower_right[0]*model->m_magnification_grid_unwrapped - pos_display_upper_left[0]*model->m_magnification_grid_unwrapped);

    cv::Mat grid_cell;
    cv::Rect rect(pos_display_upper_left[0]*model->m_magnification_grid_unwrapped,
                  pos_display_upper_left[1]*model->m_magnification_grid_unwrapped,
                  size,
                  size);
    grid_cell=img(rect);

    cv::imwrite( path_cell, grid_cell );

  }


  //Set the background and camera back to default
  renderer->SetBackground(1.0,1.0,1.0);
  renderer->SetBackground2(0.1,0.1,0.1);
  renderer->GetActiveCamera()->SetParallelProjection(0);
  renderer->ResetCamera();
  this->ui->qvtkWidget->GetRenderWindow()->Render();
}



void Visualizer::render_grid_wrapped(){
  if (model->m_points_wrapped.empty()){
    return;
  }

  //if it has not been unwrapped, unwrap it. But the mesh will stay wrapped
  model->m_is_unwrapped=false;
  if (model->m_points_unwrapped.empty() && !model->m_points_wrapped.empty()){
    model->compute_unwrap2();
    // model->write_points_to_mesh();  //Need to add the computed unwraps to the mesh so that the grid is correct
    model->create_grid();
  }
  updateView();


  //If the path does not exist, create it
  std::string path_sufix= "/Grid_wrapped";
  std::string path=model->m_path_global+ path_sufix;
  std::cout << "Path to write in is " << path << std::endl;
  if (!boost::filesystem::is_directory(path) ){
    // std::cout << "path does not exist, creating it" << std::endl;
    if (!boost::filesystem::create_directories(path)){
      std::cout << "Path could not be created or accessed" << std::endl;
      return;
    }else{
      // std::cout << "created the path correctly" << std::endl;
    }
  }

  model->wrap_grid();


  //middle point
  for (size_t i = 0; i < model->m_grid_wrapped.size(); i++) {
    row_type mid_point(3, 0.0);

    mid_point[0]= (model->m_grid_wrapped[i][0][0] + model->m_grid_wrapped[i][2][0] )/2.0;
    mid_point[1]= (model->m_grid_wrapped[i][0][1] + model->m_grid_wrapped[i][2][1] )/2.0;
    mid_point[2]= (model->m_grid_wrapped[i][0][2] + model->m_grid_wrapped[i][2][2] )/2.0;


    //Point 0,1,2,3 follow a clockwise order in the rectangle that defines a cell
    // draw_line(model->m_grid_wrapped[i][0].data(), model->m_grid_wrapped[i][1].data());
    // draw_line(model->m_grid_wrapped[i][1].data(), model->m_grid_wrapped[i][2].data());
    // draw_line(model->m_grid_wrapped[i][2].data(), model->m_grid_wrapped[i][3].data());
    // draw_line(model->m_grid_wrapped[i][3].data(), model->m_grid_wrapped[i][0].data());

    row_type normal(3, 0.0);
    //To get  normal going inside the chimney we need a vector from point 1->0 and from points 1->2. Cross product
    //therefore we do 0-1 and 2-1

    //To get  normal going inside the chimney we need a vector from point min->0 and from points min->1. Cross product
    //therefore we do 0-mid and 1-mid

    Eigen::Vector3f vec1( model->m_grid_wrapped[i][1][0] - mid_point[0],
                          model->m_grid_wrapped[i][1][1] - mid_point[1],
                          model->m_grid_wrapped[i][1][2] - mid_point[2]);

    Eigen::Vector3f vec2( model->m_grid_wrapped[i][2][0] - mid_point[0],
                          model->m_grid_wrapped[i][2][1] - mid_point[1],
                          model->m_grid_wrapped[i][2][2] - mid_point[2]);

    Eigen::Vector3f cross = vec2.cross(vec1);
    double cross_vec[3];
    cross_vec[0]=cross[0];
    cross_vec[1]=cross[1];
    cross_vec[2]=cross[2];


    //move the camera a certain distance away from the plane of the cell
    row_type pos_camera(3);

    int dist=10;
    pos_camera[0]= mid_point[0] + dist*cross[0];
    pos_camera[1]= mid_point[1] + dist*cross[1];
    pos_camera[2]= mid_point[2] + dist*cross[2];

    // draw_sphere(pos_camera[0],pos_camera[1],pos_camera[2], 0.0, 0.0, 1.0 );

    renderer->GetActiveCamera()->SetPosition (pos_camera[0],
                                              pos_camera[1],
                                              pos_camera[2]);

    renderer->GetActiveCamera()->SetFocalPoint(mid_point[0], mid_point[1], mid_point[2]);
    renderer->GetActiveCamera()->SetViewUp(0, 0, 1);


    double bounds[6]= {0};;
    calculate_bounds(model->m_grid_wrapped[i], bounds);
    renderer->ResetCamera(bounds);
    this->ui->qvtkWidget->GetRenderWindow()->Render();


    std::string file_name= "/grid_w_" + std::to_string(i) + ".png";
    path=model->m_path_global+ path_sufix + file_name;

    render_to_file(path,model->m_magnification_grid_wrapped);

    //read image
    //project 4 corners in display coords
    //get their bounding rectangle
    //draw a red rectangle


    //Read and crop
    cv::Mat img =cv::imread(path);

    matrix_type corners_display(model->m_grid_wrapped[i].size());
    for (size_t point_idx = 0; point_idx < model->m_grid_wrapped[i].size(); point_idx++) {
      corners_display[point_idx].resize(model->m_grid_wrapped[i][point_idx].size());
      world_to_display(model->m_grid_wrapped[i][point_idx].data(),corners_display[point_idx].data());

      //now each coorindate will be multiplier by the magnifiction (also multiply the z even if it doesnt matter)
      for (size_t coord = 0; coord < corners_display[point_idx].size(); coord++) {
        corners_display[point_idx][coord]=corners_display[point_idx][coord] * model->m_magnification_grid_wrapped;
      }
    }


    std::vector<cv::Point2f> corners_cv(corners_display.size());
    for (size_t point_idx = 0; point_idx < corners_cv.size(); point_idx++) {
      corners_cv[point_idx].x=corners_display[point_idx][0];
      corners_cv[point_idx].y=corners_display[point_idx][1];
    }

    cv::Rect rect= cv::boundingRect(corners_cv);

    cv::rectangle(img, rect, cv::Scalar(0,0,255),1*model->m_magnification_grid_wrapped);
    cv::imwrite( path, img );

  }


  this->ui->qvtkWidget->GetRenderWindow()->Render();
  std::cout << "finished rendering walls" << std::endl;

}



void Visualizer::render_walls(){

  if (model->m_points_wrapped.empty()){
    return;
  }

  //if it has not been unwrapped, unwrap it. But the mesh will stay wrapped
  model->m_is_unwrapped=false;
  if (model->m_points_unwrapped.empty() && !model->m_points_wrapped.empty()){
    model->compute_unwrap2();
    // model->write_points_to_mesh();  //Need to add the computed unwraps to the mesh so that the grid is correct
    model->create_grid();
  }
  updateView();


  //If the path does not exist, create it
  std::string path_sufix= "/Walls";
  std::string path=model->m_path_global+ path_sufix;
  std::cout << "Path to write in is " << path << std::endl;
  if (!boost::filesystem::is_directory(path) ){
    // std::cout << "path does not exist, creating it" << std::endl;
    if (!boost::filesystem::create_directories(path)){
      std::cout << "Path could not be created or accessed" << std::endl;
      return;
    }else{
      // std::cout << "created the path correctly" << std::endl;
    }
  }



  for (size_t i = 0; i < model->planes.size(); i++) {
    int clust_idx= model->planes[i].index_cluster;
    double angle=0.0;

    //roate the mesh
    vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
    angle= interpolate (model->planes[i].angle, 0.0, 1.0,  -M_PI, M_PI);
    angle= angle * 180.0/M_PI;
    trans->RotateZ(angle);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    #if VTK_MAJOR_VERSION <= 5
        transformFilter->SetInputConnection(model->m_wall->GetProducerPort());
    #else
        transformFilter->SetInputData(model->m_wall);
    #endif


    transformFilter->SetTransform(trans);
    transformFilter->Update();
    //Get the new points and the new normals
    model->m_wall=transformFilter->GetOutput();
    model->read_info(0);
    updateView();


    //Rotate the clustered cloud
    Eigen::Affine3f trans_cloud = Eigen::Affine3f::Identity();
    double r = interpolate (model->planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
    trans_cloud.rotate (Eigen::AngleAxisf (r, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*(model->clustered_clouds_original[clust_idx]),
                              *(model->clustered_clouds_original[clust_idx]), trans_cloud);



    //from the inliers of that cloud get the bounds
    double bounds[6];

    bounds[0]=std::numeric_limits<double>::max();   //x_min
    bounds[1]=-std::numeric_limits<double>::min();  //x_max
    bounds[2]=std::numeric_limits<double>::max();   //y_min
    bounds[3]=-std::numeric_limits<double>::min();  //y_max
    bounds[4]=std::numeric_limits<double>::max();   //z_min
    bounds[5]=-std::numeric_limits<double>::min();  //z_max
    for (size_t in_idx = 0; in_idx < model->m_inliers_vec[clust_idx]->indices.size (); in_idx++) {

      double x=model->clustered_clouds_original[clust_idx]->points[ model->m_inliers_vec[clust_idx]->indices[in_idx]].x;
      double y=model->clustered_clouds_original[clust_idx]->points[ model->m_inliers_vec[clust_idx]->indices[in_idx]].y;
      double z=model->clustered_clouds_original[clust_idx]->points[ model->m_inliers_vec[clust_idx]->indices[in_idx]].z;

      //x min
      if (x < bounds[0])
        bounds[0]=x;
      //X_max
      if (x > bounds[1])
        bounds[1]=x;
      //y min
      if (y < bounds[2])
        bounds[2]=y;
      //y_max
      if (y > bounds[3])
        bounds[3]=y;
      //z min
      if (z < bounds[4])
        bounds[4]=z;
      //z_max
      if (z > bounds[5])
        bounds[5]=z;

    }


    row_type bounds_vec(std::begin(bounds), std::end(bounds));
    draw_cell(bounds_vec, 1.0, 0.0, 0.0);

    //Set the camera
    renderer->GetActiveCamera()->SetPosition (0.0, 0.0, 0.0);
    renderer->GetActiveCamera()->SetViewUp (0.0, 0.0, 1.0);
    row_type focal_point(3);
    focal_point[0]= (bounds[0]+  bounds[1])/2.0;
    focal_point[1]= (bounds[2]+  bounds[3])/2.0;
    focal_point[2]= (bounds[4]+  bounds[5])/2.0;
    draw_sphere(focal_point[0],focal_point[1], focal_point[2]);
    renderer->GetActiveCamera()->SetFocalPoint (focal_point[0],focal_point[1], focal_point[2]);
    renderer->ResetCamera(bounds);
    // this->ui->qvtkWidget->GetRenderWindow()->Render();

    std::string file_name= "/wall_" + std::to_string(i) + ".png";
    path=model->m_path_global+ path_sufix + file_name;

    render_to_file(path,model->m_magnification_walls);


    //Rotate them back
    vtkSmartPointer<vtkTransform> trans_back = vtkSmartPointer<vtkTransform>::New();
    angle= interpolate (model->planes[i].angle, 0.0, 1.0,  -M_PI, M_PI);
    angle= angle * 180.0/M_PI;
    trans_back->RotateZ(-angle);

    // std::cout << "+rotating angle: back " << -angle << std::endl;
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter_back = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    #if VTK_MAJOR_VERSION <= 5
      transformFilter_back->SetInputConnection(model->m_wall->GetProducerPort());
    #else
      transformFilter_back->SetInputData(model->m_wall);
    #endif

    transformFilter_back->SetTransform(trans_back);
    transformFilter_back->Update();
    //Get the new points and the new normals
    model->m_wall=transformFilter_back->GetOutput();
    model->read_info(0);
    updateView();



    Eigen::Affine3f trans_cloud_back = Eigen::Affine3f::Identity();
    double r_back = interpolate (model->planes[i].angle, 0.0, 1.0, -M_PI, M_PI);
    trans_cloud_back.rotate (Eigen::AngleAxisf (-r_back, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*(model->clustered_clouds_original[clust_idx]),
                              *(model->clustered_clouds_original[clust_idx]), trans_cloud_back);

  }

}



void Visualizer::on_loadFileConfigButton_clicked(){
  std::cout << "loadfileconfig" << std::endl;

  //m_config->numWallsText->setText(QString::number(model->m_num_walls));
  m_config->pathText->setText( QString::fromUtf8(model->m_path_global.data()) );

  m_config->fullImgMagText->setText(QString::number(model->m_magnification_full_img));
  m_config->gridUnwrappedMagText->setText(QString::number(model->m_magnification_grid_unwrapped));
  m_config->gridWrappedMagText->setText(QString::number(model->m_magnification_grid_wrapped));
  m_config->wallsMagText->setText(QString::number(model->m_magnification_walls));

  m_config->exec();
}

void Visualizer::on_renderToFileButton_clicked(){
  std::cout << "rendering everything into files" << std::endl;
  std::cout << "path is " << model->m_path_global << std::endl;
  std::cout << "render full img is " << model->m_render_full_img << std::endl;
  std::cout << "render grid unwrapped is  " << model->m_render_grid_unwrapped << std::endl;
  std::cout << "render grid wrapped is " << model->m_render_grid_wrapped << std::endl;
  std::cout << "render walls is " << model->m_render_walls << std::endl;

  if (model->m_render_full_img){
    render_full_img();
  }

  if (model->m_render_grid_unwrapped){
    render_grid_unwrapped();
  }

  if (model->m_render_grid_wrapped){
    render_grid_wrapped();
  }

  if (model->m_render_walls){
    render_walls();
  }

}


//-------OPTIONS-------------------------

void Visualizer::on_numWallsText_textChanged(const QString & text){
  std::cout << "setting the num of walls to " <<  text.toStdString() << std::endl;
  int num= atoi (text.toStdString().data());
  model->mesh->set_nr_walls(  atoi (text.toStdString().data())  ) ;
}

void Visualizer::on_experimentalLoadingcheckBox_clicked(){
  // std::cout << "eperimental loading" << std::endl;

  if (m_config->experimentalLoadingcheckBox->isChecked()){
    std::cout << "using experimental loading" << std::endl;
    model->m_experiemental_loading=true;
  }else{
    std::cout << "not using experimental loading" << std::endl;
    model->m_experiemental_loading=false;
  }

}

void Visualizer::on_fixOrientationcheckBox_clicked(){
  std::cout << "setting fix orientation" << std::endl;

  // if (m_config->fixOrientationcheckBox->isChecked()){
  //   std::cout << "fixing orientation fo mesh" << std::endl;
  //   model->m_fix_orientation=true;
  // }else{
  //   std::cout << "not fixing orientation fo mesh" << std::endl;
  //   model->m_fix_orientation=false;
  // }

}


void Visualizer::on_deformWallscheckBox_clicked(){

  if (m_config->deformWallscheckBox->isChecked()){
    std::cout << "will deform walls" << std::endl;
    model->m_deform_walls=true;
  }else{
    std::cout << "will not deform walls" << std::endl;
    model->m_deform_walls=false;
  }
}

void Visualizer::on_clearUnwrapButton_clicked(){
  std::cout << "clearing unwrap" << std::endl;
  model->m_points_unwrapped.clear();
  model->m_is_unwrapped=false;
  // model->m_deleted_streached_trigs=false;  //TODO: If you put it to false it segments faults
  updateView();
}

void Visualizer::on_recomputeColorsButton_clicked(){
  // std::string color_combo_box_text= ui->colorComboBox->currentText().toStdString();
  on_colorComboBox_currentIndexChanged(ui->colorComboBox->currentText());
}

void Visualizer::on_pathText_textChanged(const QString & text){
  std::cout << "setting the global path to of rendering to " <<  text.toStdString() << std::endl;
  model->m_path_global=text.toStdString();
}

void Visualizer::on_renderFullImgcheckBox_clicked(){
  if (m_config->renderFullImgcheckBox->isChecked()){
    std::cout << "will render full img" << std::endl;
    model->m_render_full_img=true;
  }else{
    std::cout << "will not render full img" << std::endl;
    model->m_render_full_img=false;
  }
}

void Visualizer::on_renderGridUnwrappedcheckBox_clicked(){
  if (m_config->renderGridUnwrappedcheckBox->isChecked()){
    std::cout << "will render unwrapped grid img" << std::endl;
    model->m_render_grid_unwrapped=true;
  }else{
    std::cout << "will not render unwrapped grid img" << std::endl;
    model->m_render_grid_unwrapped=false;
  }
}

void Visualizer::on_renderGridWrappedcheckBox_clicked(){

  if (m_config->renderGridWrappedcheckBox->isChecked()){
    std::cout << "will render wrapped grid img" << std::endl;
    model->m_render_grid_wrapped=true;
  }else{
    std::cout << "will not render wrapped grid img" << std::endl;
    model->m_render_grid_wrapped=false;
  }

}

void Visualizer::on_renderWallscheckBox_clicked(){

  if (m_config->renderWallscheckBox->isChecked()){
    std::cout << "will render walls" << std::endl;
    model->m_render_walls=true;
  }else{
    std::cout << "will not render walls" << std::endl;
    model->m_render_walls=false;
  }
}

void Visualizer::on_fullImgMagText_textChanged(const QString & text){
  int num= atoi (text.toStdString().data());
  if (num>20)
    num=20;
  std::cout << "setting the full img magnification to to " << num << std::endl;
  model->m_magnification_full_img = num;
}
void Visualizer::on_gridUnwrappedMagText_textChanged(const QString & text){
  int num= atoi (text.toStdString().data());
  if (num>20)
    num=20;
  std::cout << "setting the grid unwrapped magnification to to " << num << std::endl;
  model->m_magnification_grid_unwrapped = num;
}
void Visualizer::on_gridWrappedMagText_textChanged(const QString & text){
  int num= atoi (text.toStdString().data());
  if (num>20)
    num=20;
  std::cout << "setting the grid wrapped magnification to to " << num << std::endl;
  model->m_magnification_grid_wrapped = num;
}
void Visualizer::on_wallsMagText_textChanged(const QString & text){
  int num= atoi (text.toStdString().data());
  if (num>20)
    num=20;
  std::cout << "setting the walls magnification to to " << num << std::endl;
  model->m_magnification_walls = num;
}

void Visualizer::on_aboveThreshText_textChanged(const QString & text){
  double num= atof (text.toStdString().data());
  std::cout << "setting the above thresh to " <<  num << std::endl;
  model->mesh->set_above_tresh(  num  ) ;
  // model->mesh->compute_depth_defects_colors();
}

void Visualizer::on_belowThreshText_textChanged(const QString & text){
  double num= atof (text.toStdString().data());
  std::cout << "setting below thresh to " <<  num << std::endl;
  model->mesh->set_below_tresh(  num  ) ;
  // model->mesh->compute_depth_defects_colors();
}
