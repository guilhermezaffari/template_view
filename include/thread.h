#ifndef THREAD_H
#define THREAD_H
#include <QtCore>
#include <ros/ros.h>
#include <QObject>
#include <dolphin_slam/ActiveLocalViewCells.h>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <dolphin_slam/LocalViewNetwork.h>
#include <dolphin_slam/Error.h>
#include <dolphin_slam/ExecutionTime.h>

#include <iostream>

using namespace std;
using namespace boost;

class Thread : public QThread
{
    Q_OBJECT

public:
    explicit Thread(QObject *parent = 0);
    void run();
    void localViewCallback(const dolphin_slam::ActiveLocalViewCellsConstPtr & msg);
    void localisationErrorCallback(const dolphin_slam::ErrorConstPtr & msg);
    void executionTimeCallback(const dolphin_slam::ExecutionTimeConstPtr &msg);


    string getFilename();

private:
    ros::NodeHandle node_handle_;
    string filename_;
    void loadConfig();


signals:
    void ros_closed();


    void local_view_received(double, double);
    void error_received(double, double,double);

    void lv_time_received(double, double);
    void em_time_received(double, double);
    void pc_time_received(double, double);


};

#endif // THREAD_H
