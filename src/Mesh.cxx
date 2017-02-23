#include "Mesh.h"
#include <iostream>

Mesh::Mesh():
  m_wall(vtkSmartPointer<vtkPolyData>::New())
{

}
