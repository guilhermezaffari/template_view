//#include <QtGui/QApplication>

#include <QtWidgets/QApplication>
#include "mainwindow.h"
#include <string>
#include <iostream>

using namespace std;
int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    ros::init(argc, argv, "template_view_node");

    MainWindow w;
    w.show();

    return a.exec();
}

