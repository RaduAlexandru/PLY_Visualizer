
cmake_minimum_required(VERSION 2.8)

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()

PROJECT(Visualizer)

set(CMAKE_BUILD_TYPE Debug)




find_package(PCL  REQUIRED )
find_package(VTK REQUIRED)
find_package(MPI REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include(${VTK_USE_FILE})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5Widgets REQUIRED QUIET)
else()
  find_package(Qt4 REQUIRED)
  include(${QT_USE_FILE})
endif()


file(GLOB CXX_FILES *.cxx)
file(GLOB UI_FILES *.ui)
file(GLOB QT_WRAP *.h)








if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  qt5_wrap_ui(UISrcs ${UI_FILES} )
  # CMAKE_AUTOMOC in ON so the MOC headers will be automatically wrapped.
  add_executable(Visualizer MACOSX_BUNDLE
    ${CXX_FILES} ${UISrcs} ${QT_WRAP})
  qt5_use_modules(Visualizer Core Gui)
  target_link_libraries(Visualizer ${Boost_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
else()
  QT4_WRAP_UI(UISrcs ${UI_FILES})
  QT4_WRAP_CPP(MOCSrcs ${QT_WRAP})
  add_executable(Visualizer MACOSX_BUNDLE ${CXX_FILES} ${UISrcs} ${MOCSrcs})

  #for C++11
  target_compile_features(Visualizer PRIVATE cxx_range_for )

  if(VTK_LIBRARIES)
    if(${VTK_VERSION} VERSION_LESS "6")
      target_link_libraries(Visualizer ${Boost_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} QVTK)
    else()
      target_link_libraries(Visualizer ${Boost_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
    endif()
  else()
    target_link_libraries(Visualizer ${Boost_LIBRARIES} vtkHybrid QVTK vtkViews ${QT_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
  endif()
endif()
