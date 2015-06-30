#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>
#include <QDebug>
#include <QDoubleValidator>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <cstdlib>

#include "BasicOperations.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_concatenate_clicked()
{
    m_dialog.exec();
}

void MainWindow::on_showMesh_clicked()
{
    #ifdef __linux__ 
    std::string command("./mesh_viewer output.ply");
    #else
    std::string command("mesh_viewer.exe output.ply");
    #endif
    qDebug() << "[INF] Show Mesh Command: " << command.c_str();
    std::system(command.c_str());

}

void MainWindow::on_startScan_clicked()
{
    #ifdef __linux__ 
    std::string command("./grab_points");
    #else
    std::string command("grab_points.exe");
    #endif
    qDebug() << "[INF] Start Scan Command: " << command.c_str();
    std::system(command.c_str());
}

void MainWindow::on_crop_clicked()
{
    #ifdef __linux__   /* Linux */
    std::string command("./process_points output");
    #else
    std::string command("process_points.exe output");
    #endif
    qDebug() << "[INF] Crop Clouds Command: " << command.c_str();
    std::system(command.c_str());
}

void MainWindow::on_showPoints_clicked()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("output.pcd", *cloud) == -1) //* load the file
    {
        std::string msg("[ERR] Couldn't read file output.pcd\n");
        qDebug() << msg.c_str();
        return;
    }
    
    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
    
    while (!viewer.wasStopped()) {
        viewer.showCloud(cloud);
    }
}
