#include "InteractorPointPicker.h"

InteractorPointPicker::InteractorPointPicker():
wall(vtkSmartPointer<vtkPolyData>::New()),
ids(vtkSmartPointer<vtkIdTypeArray>::New())
{
  std::cout << "InteractorPointPicker CONSTRUCT" << std::endl;
  //this->PointPicker = vtkSmartPointer<vtkPointPicker>::New();
  ids->SetNumberOfComponents(1);

  add_point_allowed=true;
  dragging=false;
  last_point[0]=0;
  last_point[1]=0;
  last_point[2]=0;


  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
  boost::thread t(boost::bind(&InteractorPointPicker::allow_add_point, this));

}

InteractorPointPicker* InteractorPointPicker::New(){
  InteractorPointPicker* returnValue = new InteractorPointPicker();
  return returnValue;

}


void InteractorPointPicker::OnLeftButtonDown()
{
  // std::cout << "Picking pixel: " << this->Interactor->GetEventPosition()[0] << " " << this->Interactor->GetEventPosition()[1] << std::endl;
  // this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0],
  //                    this->Interactor->GetEventPosition()[1],
  //                    0,  // always zero.
  //                    this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
  // double picked[3];
  // this->Interactor->GetPicker()->GetPickPosition(picked);
  // std::cout << "Picked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;

  if (*selecting_defects){
    dragging=true;
    OnMouseMove();
  }


  // //attempt 3
  // vtkIdType point_id=0;
  // int* clickPos = this->GetInteractor()->GetEventPosition();
  // vtkSmartPointer<vtkPointPicker>  picker = vtkSmartPointer<vtkPointPicker>::New();
  //
  // // Pick from this location.
  // picker->Pick(clickPos[0], clickPos[1], 0, this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
  // double* pos = picker->GetPickPosition();
  // std::cout << "Pick position (world coordinates) is: "
  //           << pos[0] << " " << pos[1]
  //           << " " << pos[2] << std::endl;
  // point_id =  picker->GetPointId();
  // std::cout << "id is: "<< point_id << std::endl;
  // if (point_id!=-1)
  //   ids->InsertNextValue(point_id);
  //
  //
  //
  //
  //
  // //Attempt to change color
  // std::vector<double> curTuple(3);
  // curTuple[0]=255.0;
  // curTuple[1]=0.0;
  // curTuple[2]=0.0;
  // //point_id=0;
  // auto colors= wall->GetPointData()->GetScalars();
  // if (colors==nullptr){
  //   std::cout << "interactor: wall has no colors" << std::endl;
  // }else{
  //   std::cout << "interactor: wall DOES has colors" << std::endl;
  //   if (point_id!=-1){
  //     wall->GetPointData()->GetScalars()->SetTuple(point_id, curTuple.data());
  //     //wall->GetPointData()->SetScalars(  wall->GetPointData()->GetScalars() );
  //     wall->Modified();
  //     //this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();
  //   }
  // }
  // std::cout << "interactor: added color" << std::endl;
  // //wall->Modified();






  // //Extraction, works but takes too long
  // //Selected with those ids
  // vtkSmartPointer<vtkSelectionNode> selectionNode =
  //   vtkSmartPointer<vtkSelectionNode>::New();
  // selectionNode->SetFieldType(vtkSelectionNode::POINT);
  // selectionNode->SetContentType(vtkSelectionNode::INDICES);
  // selectionNode->SetSelectionList(ids);
  // selectionNode->GetProperties()->Set(vtkSelectionNode::CONTAINING_CELLS(), 1);
  //
  // vtkSmartPointer<vtkSelection> selection =
  //   vtkSmartPointer<vtkSelection>::New();
  // selection->AddNode(selectionNode);
  //
  //
  // vtkSmartPointer<vtkExtractSelection> extractSelection =
  //   vtkSmartPointer<vtkExtractSelection>::New();
  //
  // extractSelection->SetInputConnection(0, wall->GetProducerPort());
  //
  //   extractSelection->SetInput(1, selection);
  // extractSelection->Update();
  //
  //
  // // In selection
  // vtkSmartPointer<vtkUnstructuredGrid> selected =
  //   vtkSmartPointer<vtkUnstructuredGrid>::New();
  // selected->ShallowCopy(extractSelection->GetOutput());
  //
  // std::cout << "There are " << selected->GetNumberOfPoints()
  //           << " points in the selection." << std::endl;
  // std::cout << "There are " << selected->GetNumberOfCells()
  //           << " cells in the selection." << std::endl;









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


void InteractorPointPicker::OnLeftButtonUp()
{
  if (*selecting_defects){
    last_point[0]=0;
    last_point[1]=0;
    last_point[2]=0;
    dragging=false;
  }
  vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}


void InteractorPointPicker::OnMouseMove(){
    //std::cout << "mouse moving" << std::endl;

    //std::cout << "moving with selection mode: " << *selecting_defects << std::endl;

    if (*selecting_defects && dragging){
      //std::cout << "paining" << std::endl;
      add_point();
      //paint();
    }else{
      vtkInteractorStyleTrackballCamera::OnMouseMove();
    }

    //OnLeftButtonDown();
    //add_point();
    //vtkInteractorStyleTrackballCamera::OnMouseMove();
    //this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();
    //vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void InteractorPointPicker::allow_add_point()
{
  while(true){
      //wall->Modified();

      // int num =ids->GetSize();
      // if (num!=0){
      //   this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();
      // }
      // std::cout << "TICK:::number of points" << num << std::endl;

      //std::cout << "Tick: we allow adding a points!\n";

      add_point_allowed=true;
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
}


void InteractorPointPicker::add_point(){
  //add a point
  if (add_point_allowed){
    //std::cout << "adding point" << std::endl;


    vtkIdType point_id=0;
    int* clickPos = this->GetInteractor()->GetEventPosition();
    vtkSmartPointer<vtkPointPicker>  picker = vtkSmartPointer<vtkPointPicker>::New();

    // Pick from this location.
    picker->Pick(clickPos[0], clickPos[1],0,this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
    double* pos = picker->GetPickPosition();
    point_id    = picker->GetPointId();
    std::cout << "Pick_vertex is: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    //std::cout << "id is: "<< point_id << std::endl;
    if (point_id!=-1){
      ids->InsertNextValue(point_id);
    }else{
      std::cout << "no point in mesh selected" << std::endl;
      return;
    }



    //Get scale
    //this->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()
    // double* scale;
    // scale=this->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActors()->GetNextActor()->GetScale();







    //Make a shpere at that point
    if (last_point[0]==0 && last_point[1]==0 && last_point[2]==0){
      last_point[0]=pos[0];
      last_point[1]=pos[1];
      last_point[2]=pos[2];
    }


    // vtkSmartPointer<vtkSphereSource> sphereSource =
    // vtkSmartPointer<vtkSphereSource>::New();
    // sphereSource->SetCenter(pos[0], pos[1], pos[2]);
    // sphereSource->SetRadius(0.05);
    // sphereSource->SetPhiResolution(30);
    // sphereSource->SetThetaResolution(30);
    //
    // vtkSmartPointer<vtkPolyDataMapper> mapper =
    //   vtkSmartPointer<vtkPolyDataMapper>::New();
    // mapper->SetInputConnection(sphereSource->GetOutputPort());
    //
    // vtkSmartPointer<vtkActor> actor =
    //   vtkSmartPointer<vtkActor>::New();
    // actor->SetMapper(mapper);
    // //actor->GetProperty()->SetColor(1.0, 1.0, 0.0); //(R,G,B)
    // actor->GetProperty()->SetColor(0.9, 0.5, 0.0); //(R,G,B)
    //
    // //this->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    // // this->GetInteractor()->GetRenderWindow()->Render();
    // // this->HighlightProp(NULL);







    //makea point insted of a sphere
    vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
    const float p[3] = {1.0, 2.0, 3.0};

    // Create the topology of the point (a vertex)
    vtkSmartPointer<vtkCellArray> vertices =
      vtkSmartPointer<vtkCellArray>::New();
    vtkIdType pid[1];
    pid[0] = points->InsertNextPoint(pos);
    vertices->InsertNextCell(1,pid);

    // Create a polydata object
    vtkSmartPointer<vtkPolyData> point =
      vtkSmartPointer<vtkPolyData>::New();

    // Set the points and vertices we created as the geometry and topology of the polydata
    point->SetPoints(points);
    point->SetVerts(vertices);

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInput(point);


    vtkSmartPointer<vtkActor> actor =
      vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(5);
    //actor->GetProperty()->SetColor(0.9, 0.5, 0.0); //(R,G,B)
    actor->GetProperty()->SetColor(0.137, 0.39, 0.40); //(R,G,B)
    this->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);






    //make a line to connect the last sphere and the newly created one
    vtkSmartPointer<vtkLineSource> lineSource =
    vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(last_point);
    lineSource->SetPoint2(pos);
    lineSource->Update();

    vtkSmartPointer<vtkPolyDataMapper> line_mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
    line_mapper->SetInputConnection(lineSource->GetOutputPort());
    vtkSmartPointer<vtkActor> line_actor =
      vtkSmartPointer<vtkActor>::New();
    line_actor->SetMapper(line_mapper);
    line_actor->GetProperty()->SetLineWidth(6);
    //line_actor->GetProperty()->SetColor(0.0, 0.0, 1.0); //(R,G,B)
    //line_actor->GetProperty()->SetColor(0.6, 0.1, 0.1); //(R,G,B)
    line_actor->GetProperty()->SetColor(0.66, 0.23, 0.22); //(R,G,B)

    this->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(line_actor);

    last_point[0]=pos[0];
    last_point[1]=pos[1];
    last_point[2]=pos[2];



    this->GetInteractor()->GetRenderWindow()->Render();
    //this->HighlightProp(NULL);
    add_point_allowed=false;
  }

}
