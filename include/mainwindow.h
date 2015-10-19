#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "thread.h"
#include "qcustomplot.h"

#include <QtWidgets>
#include <QFileDialog>




namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Thread *thread_ros;

    void setParameters(std::string nGroup,std::string kpTresh,std::string vtTresh);
    void setParameters();

public slots:
    void addLocalViewData(double time, double id);
    void addErrorData(double time, double em_error, double dr_error);
    void addPCTimeDate(double time, double duration);
    void addEMTimeDate(double time, double duration);
    void addLVTimeDate(double time, double duration);
    void onRosClosed();

private:
    Ui::MainWindow *ui;
    std::string filename;
    QCPGraph *lv_graph;
    QCPGraph *em_error_graph;
    QCPGraph *dr_error_graph;
    QCPGraph *pose_cell_time_graph_;
    QCPGraph *experience_map_time_graph;
    QCPGraph *local_view_time_graph;




    void createGraphs();
};

#endif // MAINWINDOW_H
