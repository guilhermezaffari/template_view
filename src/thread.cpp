#include "thread.h"
#include "mainwindow.h"

Thread::Thread(QObject *parent) :
    QThread(parent)
{

}


void Thread::localViewCallback(const dolphin_slam::ActiveLocalViewCellsConstPtr & msg)
{
    static bool first_time = true;
    static ros::Time first_stamp;
    double time;

    if(first_time)
    {
        first_stamp = msg->header.stamp;
        first_time = false;
    }

    time = (msg->header.stamp - first_stamp).toSec();

    emit local_view_received(time,static_cast<double>(msg->most_active_cell_));
}

void Thread::localisationErrorCallback(const dolphin_slam::ErrorConstPtr & msg)
{
    static bool first_time = true;
    static ros::Time first_stamp;
    double time;

    if(first_time)
    {
        first_stamp = msg->header.stamp;
        first_time = false;
    }

    time = (msg->header.stamp - first_stamp).toSec();

    emit error_received(time,msg->localization_error_em_,msg->localization_error_dr_);
}

void Thread::executionTimeCallback(const dolphin_slam::ExecutionTimeConstPtr &msg)
{
    static bool first_time = true;
    static ros::Time first_stamp;
    double time;

    if(first_time)
    {
        first_stamp = msg->header.stamp;
        first_time = false;
    }

    time = (msg->header.stamp - first_stamp).toSec();

    if(msg->module == "lv")
    {
        emit lv_time_received(time,msg->iteration_time);
    }
    else if (msg->module == "pc")
    {
        emit pc_time_received(time,msg->iteration_time);

    }
    else if(msg->module == "em")
    {
        emit em_time_received(time,msg->iteration_time);

    }
    else
    {
        ROS_FATAL_STREAM("O identificador do módulo não é válido. Identificador recebido = " << msg->module);
    }
}

void Thread::loadConfig()
{
    string dictionary_name;
    int surf_threshold;
    double match_threshold;

    node_handle_.param<double>("match_threshold",match_threshold,0.03);
    node_handle_.param<string>("bow/dictionary_name",dictionary_name,"features100/100.xml");

    char_separator<char> sep("/.");
    tokenizer< char_separator<char> > tokens(dictionary_name, sep);
    BOOST_FOREACH (const string& t, tokens) {
        if(t != "xml"){
            filename_ += t + "_";
        }
    }

    filename_ += boost::lexical_cast<string>(static_cast<float>(match_threshold)) + ".png";

    //cout << "filename: " << filename_ << endl;

}

string Thread::getFilename()
{
    return filename_;
}

void Thread::run(){

    ros::Subscriber lv_sub = node_handle_.subscribe<dolphin_slam::ActiveLocalViewCells>("/local_view_cells",1000,&Thread::localViewCallback,this);
   ros::Subscriber error_sub = node_handle_.subscribe<dolphin_slam::Error>("/error",1000,&Thread::localisationErrorCallback,this);
    ros::Subscriber time_sub = node_handle_.subscribe<dolphin_slam::ExecutionTime>("/execution_time",1000,&Thread::executionTimeCallback,this);

    loadConfig();

    ros::spin();

    cout << "Saindo da thread" << endl;

    emit ros_closed();

}
