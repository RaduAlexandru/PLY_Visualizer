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
#include <vtkExtractSelection.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdTypeArray.h>
#include <vtkInformation.h>
#include <vtkVersion.h>
#include <vtkDataArray.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkLineSource.h>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>

#include <QObject>


class InteractorPointPicker :  public QObject, public vtkInteractorStyleTrackballCamera
{
     Q_OBJECT
public:
    InteractorPointPicker();
    static InteractorPointPicker* New();
    vtkTypeMacro(InteractorPointPicker, vtkInteractorStyleTrackballCamera);
    virtual void OnLeftButtonDown();
    void OnMouseMove();
    void OnLeftButtonUp();
    void print();
    void allow_add_point();
    void add_point();

    bool add_point_allowed;
    bool* selecting_defects;
    bool dragging;

    vtkSmartPointer<vtkPolyData> wall;

    vtkSmartPointer<vtkIdTypeArray> ids;
    double last_point[3];

public slots:
    void on_selectButton_clicked();

};

#endif // INTERACTORPOINTPICKER_H
