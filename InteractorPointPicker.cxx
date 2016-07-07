#include "InteractorPointPicker.h"

InteractorPointPicker::InteractorPointPicker():
wall(vtkSmartPointer<vtkPolyData>::New()),
test(0)
{
  std::cout << "InteractorPointPicker CONSTRUCT" << std::endl;
  //this->PointPicker = vtkSmartPointer<vtkPointPicker>::New();
}


void InteractorPointPicker::OnLeftButtonDown()
{
  std::cout << "Picking pixel: " << this->Interactor->GetEventPosition()[0] << " " << this->Interactor->GetEventPosition()[1] << std::endl;
  this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0],
                     this->Interactor->GetEventPosition()[1],
                     0,  // always zero.
                     this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
  double picked[3];
  this->Interactor->GetPicker()->GetPickPosition(picked);
  std::cout << "Picked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;


  //ATTEMPT1 at coloring a point
  int num;
  num=wall->GetNumberOfPoints();
  vtkSmartPointer<vtkUnsignedCharArray> colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  for (vtkIdType i = 0; i < num; i++) {
    colors->InsertNextTuple3(255.0, 0.0, 0.0);
  }
  wall->GetPointData()->SetScalars(colors);


  //Attempt 2
  vtkSmartPointer<vtkPointPicker>  picker = vtkSmartPointer<vtkPointPicker>::New();
  vtkIdType point_id=0;
  //point_id =  this->Interactor->GetPicker()->GetPicked();
  point_id =  picker->GetPointId();
  std::cout << "id is: "<< point_id << std::endl;


   vtkSmartPointer<vtkPoints> points_picked = vtkSmartPointer<vtkPoints>::New();
   points_picked= picker->GetPickedPositions();
   std::cout << "num of points picked" << points_picked->GetNumberOfPoints() << std::endl;

   picker->GetPickPosition(picked);
   std::cout << " AAAPicked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;


  std::cout << "interactor sees: " << num << std::endl;
  std::cout << "interactor sees test : " << test << std::endl;




  //attempt 3
  int* clickPos = this->GetInteractor()->GetEventPosition();

  // Pick from this location.
  picker->Pick(clickPos[0], clickPos[1], 0, this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
  double* pos = picker->GetPickPosition();
  std::cout << "Pick position (world coordinates) is: "
            << pos[0] << " " << pos[1]
            << " " << pos[2] << std::endl;
  point_id =  picker->GetPointId();
  std::cout << "aaaaid is: "<< point_id << std::endl;

  //Clip a sphere out of it
  //vtkSmartPointer<vtkSphere> sphere_cut = vtkSmartPointer<vtkSphere>::New();
  // box_cut->SetBounds (0.0, 1000, -0.005, 0.005, -1000.0, 1000.0);
  //
  // vtkSmartPointer<vtkClipPolyData> clipper= vtkSmartPointer<vtkClipPolyData>::New();
  // clipper->SetInputConnection(reader->GetOutputPort());
  // clipper->SetClipFunction(box_cut);
  // clipper->InsideOutOff();
  // clipper->Update();
  // clipper->GenerateClippedOutputOn();






  // Forward events
  vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}



InteractorPointPicker* InteractorPointPicker::New(){
  InteractorPointPicker* returnValue = new InteractorPointPicker();
  return returnValue;

}
