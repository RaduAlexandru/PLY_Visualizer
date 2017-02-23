#include <QApplication>
#include "Visualizer.h"

int main( int argc, char** argv )
{
  //QT Stuff
  QApplication app( argc, argv );
  // setlocale(LC_NUMERIC,"C");
  // setlocale(LC_ALL, "");
  setlocale(LC_ALL, "C");

  Visualizer w;
  w.show();

  return app.exec();

}
