#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //    setGeometry(100, 100, 1024, 768);

    thread_ros = new Thread(this);

    connect(thread_ros,SIGNAL(local_view_received(double, double)),this,SLOT(addLocalViewData(double,double)));
    connect(thread_ros,SIGNAL(error_received(double,double,double)),this,SLOT(addErrorData(double,double,double)));
    connect(thread_ros,SIGNAL(em_time_received(double, double)),this,SLOT(addEMTimeDate(double,double)));
    connect(thread_ros,SIGNAL(pc_time_received(double, double)),this,SLOT(addPCTimeDate(double,double)));
    connect(thread_ros,SIGNAL(lv_time_received(double, double)),this,SLOT(addLVTimeDate(double,double)));

    connect(thread_ros,SIGNAL(ros_closed()),this,SLOT(onRosClosed()));

    createGraphs();

    thread_ros->start();
}

void MainWindow::createGraphs()
{
    //! Local View Plot
    lv_graph = ui->lv_plot_->addGraph();

    lv_graph->setPen(QColor(0,0,0));
    lv_graph->setName("Most Active Local View Cell");
    lv_graph->setLineStyle(QCPGraph::lsNone);
    lv_graph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 1));

    lv_graph->rescaleAxes(true);

    ui->lv_plot_->plotLayout()->insertRow(0);
    ui->lv_plot_->plotLayout()->addElement(0, 0, new QCPPlotTitle(ui->lv_plot_, "Most Active Local View"));
    ui->lv_plot_->xAxis->setLabel("Time (s)");
    ui->lv_plot_->yAxis->setLabel("Local View ID");
    ui->lv_plot_->axisRect()->setupFullAxesBox();
    ui->lv_plot_->replot();


    dr_error_graph = ui->error_plot_->addGraph();

    //! Error Plot
    //error_graph = ui->custom_plot->addGraph();
    dr_error_graph->setName("Dead Reckoning");
    dr_error_graph->setLineStyle(QCPGraph::lsLine);
    dr_error_graph->setScatterStyle(QCPScatterStyle::ssNone);
    dr_error_graph->setPen(QColor(255,0,0));

    em_error_graph = ui->error_plot_->addGraph();

    //! Error Plot
    //error_graph = ui->custom_plot->addGraph();
    em_error_graph->setName("DolphinSLAM");
    em_error_graph->setLineStyle(QCPGraph::lsLine);
    em_error_graph->setScatterStyle(QCPScatterStyle::ssNone);
    em_error_graph->setPen(QColor(0,0,255));

    ui->error_plot_->plotLayout()->insertRow(0);
    ui->error_plot_->plotLayout()->addElement(0, 0, new QCPPlotTitle(ui->error_plot_, "Localization Errors"));
    ui->error_plot_->xAxis->setLabel("Time (s)");
    ui->error_plot_->yAxis->setLabel("Error (m)");
    ui->error_plot_->axisRect()->setupFullAxesBox();
    ui->error_plot_->replot();


    pose_cell_time_graph_ = ui->execution_time_plot_->addGraph();

    pose_cell_time_graph_->setName("Pose Cell");
    pose_cell_time_graph_->setLineStyle(QCPGraph::lsLine);
    pose_cell_time_graph_->setScatterStyle(QCPScatterStyle::ssNone);
    pose_cell_time_graph_->setPen(QColor(255,0,0));

    local_view_time_graph = ui->execution_time_plot_->addGraph();

    local_view_time_graph->setName("Local View Module");
    local_view_time_graph->setLineStyle(QCPGraph::lsLine);
    local_view_time_graph->setScatterStyle(QCPScatterStyle::ssNone);
    local_view_time_graph->setPen(QColor(0,255,0));

    experience_map_time_graph = ui->execution_time_plot_->addGraph();

    experience_map_time_graph->setName("Experience Map");
    experience_map_time_graph->setLineStyle(QCPGraph::lsLine);
    experience_map_time_graph->setScatterStyle(QCPScatterStyle::ssNone);
    experience_map_time_graph->setPen(QColor(0,0,255));

    ui->execution_time_plot_->plotLayout()->insertRow(0);
    ui->execution_time_plot_->plotLayout()->addElement(0, 0, new QCPPlotTitle(ui->execution_time_plot_, "Update Times"));
    ui->execution_time_plot_->xAxis->setLabel("Time (s)");
    ui->execution_time_plot_->yAxis->setLabel("Duration (s)");
    ui->execution_time_plot_->axisRect()->setupFullAxesBox();
    ui->execution_time_plot_->replot();



}

MainWindow::~MainWindow()
{

    delete ui;
}


void MainWindow::addLocalViewData(double time, double id)
{

    lv_graph->addData(time, id);

//    lv_graph->rescaleAxes(true);

//    ui->lv_plot_->axisRect()->setupFullAxesBox();
    ui->lv_plot_->rescaleAxes(true);
    ui->lv_plot_->replot();
}

void MainWindow::addErrorData(double time, double em_error, double dr_error)
{

    em_error_graph->addData(time, em_error);
    dr_error_graph->addData(time, dr_error);


    ui->error_plot_->rescaleAxes(true);
    ui->error_plot_->replot();

}

void MainWindow::addPCTimeDate(double time, double duration)
{
    pose_cell_time_graph_->addData(time,duration);


    ui->execution_time_plot_->rescaleAxes(true);
    ui->execution_time_plot_->replot();
}

void MainWindow::addEMTimeDate(double time, double duration)
{
    experience_map_time_graph->addData(time,duration);


    ui->execution_time_plot_->rescaleAxes(true);
    ui->execution_time_plot_->replot();
}

void MainWindow::addLVTimeDate(double time, double duration)
{
    local_view_time_graph->addData(time,duration);


    ui->execution_time_plot_->rescaleAxes(true);
    ui->execution_time_plot_->replot();
}

void MainWindow::onRosClosed()
{
    //! \todo save images

    //! Fecha a janela
    close();
}

