#include "Model.h"
#include <iostream>

Model::Model():
  num_points(0),
  point_components(0),
  radius(0.0),
  circumference(0.0),
  is_unwrapped(false),
  selecting_defects(false)
{
  std::cout << "MODEL CONSTRUCT" << std::endl;
}
