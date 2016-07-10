#include "Visualizer.h"

// This is included here because it is forward declared in
// Visualizer.h
#include "ui_Visualizer.h"

#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>


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
  QString fileName;
	fileName = QFileDialog::getOpenFileName(this,
		tr("Open File"), "./", tr("File (*.*)"));

	if (fileName.isEmpty()){
    return;
  }

  std::cout << "filename: " << fileName.toStdString() << std::endl;



  if (boost::ends_with(fileName.toStdString(), ".ply")) {
    std::cout << "reading .ply file" << std::endl;
  }else if (boost::ends_with(fileName.toStdString(), ".obj")){
    std::cout << "reading .obj file" << std::endl;
  }else{
    std::cout << "NOT VALID FORMAT" << std::endl;
    return;
  }


  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  //vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName ( fileName.toStdString().c_str() );
  reader->Update();
  //reader->GetOutput()->GetPointData()->SetNormals(NULL);

  //cut the cylinder
  vtkSmartPointer<vtkBox> box_cut = vtkSmartPointer<vtkBox>::New();
  box_cut->SetBounds (0.0, 1000, -0.005, 0.005, -1000.0, 1000.0);
  vtkSmartPointer<vtkClipPolyData> clipper= vtkSmartPointer<vtkClipPolyData>::New();
  clipper->SetInputConnection(reader->GetOutputPort());
  clipper->SetClipFunction(box_cut);
  clipper->InsideOutOff();
  clipper->Update();
  clipper->GenerateClippedOutputOn();

  model->set_mesh(clipper->GetOutput());

  ui->colorComboBox->setCurrentIndex(ui->colorComboBox->findText("RGB"));


  // //trying to detect the bug in the reader
  // std::string inputFilename= "/media/alex/Nuevo_vol/Master/SHK/Data/Chimney/result/result.ply";
  // vtkSmartPointer<vtkPLYReader> reader =
  //   vtkSmartPointer<vtkPLYReader>::New();
  // reader->SetFileName ( inputFilename.c_str() );
  // reader->Update();
  // vtkSmartPointer<vtkPolyDataMapper> mapper =
  //   vtkSmartPointer<vtkPolyDataMapper>::New();
  // mapper->SetInputConnection(reader->GetOutputPort());
  // vtkSmartPointer<vtkActor> actor =
  //   vtkSmartPointer<vtkActor>::New();
  // actor->SetMapper(mapper);
  // renderer->AddActor(actor);
  // this->ui->qvtkWidget->GetRenderWindow()->Render();


  updateView();
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
  mapper->SetInputConnection(wall->GetProducerPort());
  mapper->Update();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);


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
    model->compute_unwrap();
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
    mapper->SetInputConnection(model->grid_cells[i]->GetProducerPort());
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


void Visualizer::slotExit()
{
  qApp->exit();
}
