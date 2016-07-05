#include "Visualizer.h"

// This is included here because it is forward declared in
// Visualizer.h
#include "ui_Visualizer.h"

#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>
#include "Model.h"

// Constructor
Visualizer::Visualizer()
  //renderer(vtkSmartPointer<vtkRenderer>::New()),
  //wall(vtkSmartPointer<vtkPolyData>::New())
{
  this->ui = new Ui_Visualizer;
  this->ui->setupUi(this);



  // // Sphere
  // vtkSmartPointer<vtkSphereSource> sphereSource =
  //     vtkSmartPointer<vtkSphereSource>::New();
  // sphereSource->Update();
  // vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
  //     vtkSmartPointer<vtkPolyDataMapper>::New();
  // sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
  // vtkSmartPointer<vtkActor> sphereActor =
  //     vtkSmartPointer<vtkActor>::New();
  // sphereActor->SetMapper(sphereMapper);

  // VTK Renderer
  renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->GradientBackgroundOn();
  renderer->SetBackground(1.0,1.0,1.0);
  renderer->SetBackground2(0.1,0.1,0.1);
  // renderer->AddActor(sphereActor);

  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

  // Set up action signals and slots
  connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

}


void Visualizer::on_loadFileButton_clicked(){
  std::cout << "loading file" << std::endl;


  QString fileName;
	fileName = QFileDialog::getOpenFileName(this,
		tr("Open OBJ File"), "./", tr("OBJ File (*.ply)"));

	if (fileName.isEmpty())
		return;

   std::cout << "filename: " << fileName.toStdString() << std::endl;

  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( fileName.toStdString().c_str() );
  reader->Update();

  wall.TakeReference(reader->GetOutput());

  updateView();
}

void  Visualizer::updateView(){
  std::cout << "update view" << std::endl;

  if (wall==NULL){
    std::cout << "its nulllllll" << std::endl;Z
  }

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(wall->GetProducerPort());
  mapper->Update();

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  renderer->RemoveAllViewProps();
  renderer->AddActor(actor);
  renderer->ResetCamera();

  //this->ui->qvtkWidget->GetRenderWindow()->Render();

}

void Visualizer::slotExit()
{
  qApp->exit();
}
