#include "Visualizer.h"

// This is included here because it is forward declared in
// Visualizer.h
#include "ui_Visualizer.h"

#define NUM_WALLS 8
#define MAGNIFICATION 15


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
  m_grid_metric_actor(vtkSmartPointer<vtkTextActor>::New())
{
  model->m_num_walls=NUM_WALLS;
  this->ui = new Ui_Visualizer;
  this->ui->setupUi(this);

  this->ui->colorComboBox->addItem("Plain");
  this->ui->colorComboBox->addItem("RGB");
  this->ui->colorComboBox->addItem("IR");
  this->ui->colorComboBox->addItem("Depth");
  this->ui->colorComboBox->addItem("Curvature");



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



  // Changing interactor to a custom one to handle different mouse events
  //vtkSmartPointer<vtkPointPicker> pointPicker = vtkSmartPointer<vtkPointPicker>::New();
  vtkSmartPointer<vtkCellPicker> pointPicker = vtkSmartPointer<vtkCellPicker>::New();
  this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle( interactor );
  this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(pointPicker);
  interactor->selecting_defects=&(model->m_selecting_defects);



  renderer->GetActiveCamera()->SetParallelProjection(0);

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

  // Set up action signals and slots
  connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));
  connect(interactor, &InteractorPointPicker::right_click_pressed_signal,
          model.get(), &Model::right_click_pressed_slot );
  connect(model.get(), &Model::grid_changed_signal,
          this, &Visualizer::grid_changed_slot );


  #if VTK_MAJOR_VERSION <= 5
      std::cout << "vtk version lower than 5" << std::endl;
  #else
      std::cout << "vtk verson higher than 5" << std::endl;
  #endif

}



void Visualizer::on_loadFileButton_clicked(){
  QString file_name;
  QString selfilter = tr("Mesh (*.obj *.ply)");
  file_name = QFileDialog::getOpenFileName(this,
  	         tr("Open File"), "./", selfilter);

  if (file_name.isEmpty()){
    return;
  }

  std::cout << "filename: " << file_name.toStdString() << std::endl;


  if (boost::ends_with(file_name.toStdString(), ".ply")) {
    std::cout << "reading .ply file" << std::endl;
    std::cout << "not implemented yet" << std::endl;
  }else if (boost::ends_with(file_name.toStdString(), ".obj")){
    std::cout << "reading .obj file" << std::endl;

    std::unique_ptr<OBJReader2> obj_reader(new OBJReader2());
    obj_reader->SetFileName(file_name.toStdString());
    obj_reader->Update();

    vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
    pngReader->SetFileName (obj_reader->GetTexturePath().data() );
    pngReader->Update();

    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
    texture->SetInputConnection(pngReader->GetOutputPort());

    model->set_mesh(obj_reader->GetOutput());
    model->set_texture(texture);
  }else{
    std::cout << "NOT VALID FORMAT" << std::endl;
    return;
  }



  ui->colorComboBox->setCurrentIndex(ui->colorComboBox->findText("RGB"));
  updateView();

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

  if (this->ui->colorComboBox->currentText()=="RGB")
    actor->SetTexture(model->m_full_texture);
  actor->GetProperty()->BackfaceCullingOn();

  renderer->AddActor(actor);
  if (reset_camera==1){
    renderer->ResetCamera();
  }

  //draw_sphere(0,0,0);

  draw_text_grid();

  update_grid_view();
  this->ui->qvtkWidget->GetRenderWindow()->Render();

}


void Visualizer::on_clearButton_clicked(){
  std::cout << "clearing the view" << std::endl;
  renderer->RemoveAllViewProps();
  this->ui->qvtkWidget->GetRenderWindow()->Render();
}

void Visualizer::on_unwrapButton_clicked(){
  std::cout << "unwrapping" << std::endl;

  if (!model->m_points_wrapped.empty()){
    model->m_is_unwrapped=!model->m_is_unwrapped;
  }

  if (model->m_points_unwrapped.empty() && !model->m_points_wrapped.empty()){
    // model->compute_unwrap(); //for cylinder chimneys
    model->compute_unwrap2();
    model->write_points_to_mesh();  //Need to add the computed unwraps to the mesh so that the grid is correct
    model->create_grid();
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
  if (text.toStdString() == "IR"){
    std::cout << "calculating IR colors" << std::endl;
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


  //TODO: Before reseting camera, draw all the grids so that we ensure that they are in the view. Otherwise when cropping opencv will try to crop outside of the boundries of the mat and fail.

  //show grid
  //ResetCamera
  //hide grid

  // for (size_t i = 0; i < model->m_grid_cells_active.size(); i++) {
  //   model->m_grid_cells_active[i]=1;
  // }
  // // std::cin.get();
  // update_grid_view();
  // // std::cin.get();
  // renderer->ResetCamera();
  // // std::cin.get();
  // for (size_t i = 0; i < model->m_grid_cells_active.size(); i++) {
  //   model->m_grid_cells_active[i]=0;
  // }
  // update_grid_view();
  // // std::cin.get();
  // this->ui->qvtkWidget->GetRenderWindow()->Render();


  //Set camera
  renderer->ResetCamera();
  renderer->GetActiveCamera()->SetParallelProjection(1);
  renderer->GetActiveCamera()->Elevation(270);
  // renderer->GetActiveCamera()->Pitch(270);
  renderer->GetActiveCamera()->Roll(180);
  renderer->SetBackground(0.0,0.0,0.0);
  renderer->SetBackground2(0.0,0.0,0.0);
  this->ui->qvtkWidget->GetRenderWindow()->Render();



  //Write big image
  vtkSmartPointer<vtkRenderLargeImage> renderLarge = vtkSmartPointer<vtkRenderLargeImage>::New();
  renderLarge->SetInput(renderer);
  renderLarge->SetMagnification(MAGNIFICATION);
  std::cout << "Saving image in " << path_img << std::endl;
  vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(path_img.data());
  writer->SetInputConnection(renderLarge->GetOutputPort());
  writer->Write();



  cv::Mat img =cv::imread(path_img);


  for (size_t i = 0; i < model->m_grid.size(); i++) {
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


    draw_cell(model->m_grid[i],1,0,0);
    draw_sphere(corner_upper_left[0],corner_upper_left[1],corner_upper_left[2]);
    draw_sphere(corner_lower_right[0],corner_lower_right[1],corner_lower_right[2]);

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


    std::cout << "pos_display_upper_left: " << pos_display_upper_left[0]*MAGNIFICATION << " " << pos_display_upper_left[1]*MAGNIFICATION << " " << pos_display_upper_left[2]*MAGNIFICATION << std::endl;

    std::cout << "pos_display_lower_right: " << pos_display_lower_right[0]*MAGNIFICATION << " " << pos_display_lower_right[1]*MAGNIFICATION << " " << pos_display_lower_right[2]*MAGNIFICATION << std::endl;

    double size=fabs (pos_display_lower_right[0]*MAGNIFICATION - pos_display_upper_left[0]*MAGNIFICATION);

    cv::Mat grid_cell;
    cv::Rect rect(pos_display_upper_left[0]*MAGNIFICATION,
                  pos_display_upper_left[1]*MAGNIFICATION,
                  size,
                  size);
    grid_cell=img(rect);


    std::string path_cell= path + "grid_cell_" + std::to_string(i) + ".png";
    cv::imwrite( path_cell, grid_cell );

    // break;

  }



  // vtkInteractorObserver::ComputeWorldToDisplay (renderer, corner[0], corner[1], corner[2], pos_display);

  // std::cout << "pos_display: " << pos_display[0] << " " << pos_display[1] << " " << pos_display[2] << std::endl;


  std::cout << "finished getting grid cells" << std::endl;
}


void Visualizer::on_renderWallsButton_clicked(){
  std::cout << "rendering walls" << std::endl;
  std::string path= "/home/alex/Pictures/Renders/";
  std::string file_name = "wall.png";
  std::string path_img=path+file_name;


  updateView();
  // renderer->GetActiveCamera()->SetPosition (0.0, 0.0, 0.0);
  // renderer->GetActiveCamera()->SetFocalPoint(model->m_plane_centers[4][0],
  //                                            model->m_plane_centers[4][1],
  //                                            model->m_plane_centers[4][2]);
  //
  //
  // renderer->GetActiveCamera()->SetViewUp(0, 0, 1);
  // // renderer->ResetCamera();
  // renderer->GetActiveCamera()->Dolly (0.5);

  for (size_t i = 0; i < NUM_WALLS; i++) {
    renderer->GetActiveCamera()->SetPosition (model->m_plane_centers[i][0],
                                              model->m_plane_centers[i][1],
                                              model->m_plane_centers[i][2]);

    renderer->GetActiveCamera()->SetViewUp(0, 0, 1);
    renderer->ResetCamera();
    renderer->GetActiveCamera()->Dolly (1.5);

    this->ui->qvtkWidget->GetRenderWindow()->Render();


    //Write wall
    file_name= "wall_" + std::to_string(i) + ".png";
    path_img= path + file_name;

    vtkSmartPointer<vtkRenderLargeImage> renderLarge = vtkSmartPointer<vtkRenderLargeImage>::New();
    renderLarge->SetInput(renderer);
    renderLarge->SetMagnification(MAGNIFICATION);
    std::cout << "Saving image in " << path_img << std::endl;
    vtkSmartPointer<vtkPNGWriter> writer =
      vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(path_img.data());
    writer->SetInputConnection(renderLarge->GetOutputPort());
    writer->Write();
  }




  std::cout << "finished rendering walls" << std::endl;
}



void Visualizer::draw_grid(){
  //Need to do it by iterating twice so that the active ones are drawn on top (after) the inactive cells
  for (size_t i = 0; i < model->m_grid.size(); i++) {
    if(model->m_grid_cells_active[i]==0 && model->m_draw_grid_inactive){
      draw_cell(model->m_grid[i], 0.5, 0.5, 0.5);
    }
  }

  for (size_t i = 0; i < model->m_grid.size(); i++) {
    if(model->m_grid_cells_active[i]==1 && model->m_draw_grid_active){
      draw_cell(model->m_grid[i], 1.0, 0.0, 0.0);
    }
  }

}

void Visualizer::draw_cell(row_type bounds, double r, double g, double b){
  vtkSmartPointer<vtkOutlineSource> outlineSource = vtkSmartPointer<vtkOutlineSource>::New();

  outlineSource->SetBounds(bounds.data());
  outlineSource->Update();

  vtkPolyData* outline = outlineSource->GetOutput();

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();

  mapper->SetInputData(outline);

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(r, g, b); //(R,G,B)
  // actor->GetProperty()->BackfaceCullingOn();

  m_grid_actors.push_back(actor);

  renderer->AddActor(actor);

}

void Visualizer::draw_text_grid(){

  if (model->m_grid.size()==0)
    return;

  renderer->RemoveActor(m_grid_metric_actor);

  int* w_size;
  w_size = this->ui->qvtkWidget->GetRenderWindow()->GetSize();

  int active = std::count(model->m_grid_cells_active.begin(), model->m_grid_cells_active.end(), 1);
  int total = model->m_grid.size();
  std::string grid_metric = std::to_string (active) + "/" + std::to_string(total);
  m_grid_metric_actor->SetInput ( grid_metric.data() );

  m_grid_metric_actor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
  m_grid_metric_actor->GetPosition2Coordinate()->SetCoordinateSystemToNormalizedViewport();
  m_grid_metric_actor->SetPosition(0.9,0.0);

  // m_grid_metric_actor->GetTextProperty()->SetJustificationToRight ();
  // m_grid_metric_actor->GetTextProperty()->SetVerticalJustificationToBottom();
  //
  // m_grid_metric_actor->SetPosition ( 50, 50 );

  // m_grid_metric_actor->SetPosition ( w_size[0]-50, 30 );

  // m_grid_metric_actor->SetPosition ( w_size[0]-30, 20 );
  // m_grid_metric_actor->SetPosition2 ( 200, -100.5 );
  m_grid_metric_actor->GetTextProperty()->SetFontSize ( 24 );
  m_grid_metric_actor->GetTextProperty()->SetColor ( 0.1, 0.1, 0.1 );
  m_grid_metric_actor->GetTextProperty()->ShadowOn ();
  // textActor->GetTextProperty()->SetShadowOffset (int, int);
  renderer->AddActor2D ( m_grid_metric_actor );


  //  vtkSmartPointer<vtkCornerAnnotation> cornerAnnotation =  vtkSmartPointer<vtkCornerAnnotation>::New();
  //  cornerAnnotation->SetLinearFontScaleFactor( 2 );
  //  cornerAnnotation->SetNonlinearFontScaleFactor( 1 );
  //  cornerAnnotation->SetMaximumFontSize( 20 );
  //  cornerAnnotation->SetText( 0, "lower left" );
  //  cornerAnnotation->SetText( 1, "lower right" );
  //  cornerAnnotation->SetText( 2, "upper left" );
  //  cornerAnnotation->SetText( 3, "upper right" );
  //  cornerAnnotation->GetTextProperty()->SetColor( 1, 0, 0 );
   //
  //  renderer->AddViewProp( cornerAnnotation );

}



void Visualizer::grid_changed_slot(){
  update_grid_view();
}

void Visualizer::update_grid_view(){
  for (size_t i = 0; i < m_grid_actors.size(); i++) {
    renderer->RemoveActor(m_grid_actors[i]);
  }
  m_grid_actors.clear();

  draw_grid();
  draw_text_grid();
  this->ui->qvtkWidget->GetRenderWindow()->Render();
}
