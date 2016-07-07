#ifndef INTERACTORPOINTPICKER_H
#define INTERACTORPOINTPICKER_H
#include <iostream>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkPointPicker.h>
#include <vtkPoints.h>
#include <vtkPointPicker.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>


#include <vtkSmartPointer.h>
#include <vtkPolyData.h>


class InteractorPointPicker : public vtkInteractorStyleTrackballCamera
{
public:
    InteractorPointPicker();
    static InteractorPointPicker* New();
    vtkTypeMacro(InteractorPointPicker, vtkInteractorStyleTrackballCamera);
    virtual void OnLeftButtonDown();

    vtkSmartPointer<vtkPolyData> wall;
    int test;
};

#endif // INTERACTORPOINTPICKER_H
