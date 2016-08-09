#include <QApplication>
#include "Visualizer.h"

int main( int argc, char** argv )
{
  //QT Stuff
  QApplication app( argc, argv );
  setlocale(LC_NUMERIC,"C");

  Visualizer w;
  w.show();

  return app.exec();

}
