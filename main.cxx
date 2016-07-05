#include <QApplication>
#include "Visualizer.h"

int main( int argc, char** argv )
{
  // QT Stuff
  QApplication app( argc, argv );

  Visualizer w;
  w.show();

  return app.exec();
}
