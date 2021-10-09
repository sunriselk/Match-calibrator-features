#include "FeatureMatching.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    FeatureMatching w;
    w.show();
    return a.exec();
}
