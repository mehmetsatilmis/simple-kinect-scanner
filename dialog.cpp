#include "dialog.h"
#include "ui_dialog.h"
#include <QDebug>
#include <QString>
#include <string>
#include <sstream>

#include <pcl/pcl_exports.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog),
    pre_x(0.0),
    pre_y(0.0),
    pre_z(0.60),
    isFirstTime(true)
{
    ui->setupUi(this);
    ui->lineEdit->setText("0.0");
    ui->lineEdit_2->setText("0.0");
    ui->lineEdit_3->setText("0.9");
}

Dialog::~Dialog()
{
    delete ui;
}

double Dialog::getX()
{
    ui->lineEdit->setValidator(new QDoubleValidator(this));

    QString XMAX=ui->lineEdit->text();
    double xmax=XMAX.toDouble();

    return xmax;
}

double Dialog::getY()
{
    ui->lineEdit_2->setValidator(new QDoubleValidator(this));

    QString XMAX=ui->lineEdit_2->text();
    double xmax=XMAX.toDouble();

    return xmax;
}

double Dialog::getZ()
{
    ui->lineEdit_3->setValidator(new QDoubleValidator(this));

    QString XMAX=ui->lineEdit_3->text();
    double xmax=XMAX.toDouble();

    return xmax;
}

void Dialog::on_pushButton_clicked()
{

    pre_x = getX();
    pre_y = getY();
    pre_z = getZ();

    qDebug() << "[INF] Coordinates: x: " << pre_x << " y:" << pre_y << " z: " << pre_z;

    std::stringstream ss;

    #ifdef __linux__ 
    ss << "./concat_point " << pre_x << " " << pre_y << " " << pre_z;
    #else
    ss << "concat_point.exe " << pre_x << " " << pre_y << " " << pre_z;
    #endif

    std::string command = ss.str();

    qDebug() << "[INF] Concatenate Command: " << command.c_str();

    std::system(command.c_str());

    qDebug() << "[INF] Loading concated.pcd to show result...";;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("concated.pcd", *cloud) == -1) //* load the file
    {
        std::string msg("[ERR] Couldn't read file concated.pcd\n");
        qDebug() << msg.c_str();
        return;
    }

    qDebug() << "[INF] concated.pcd loaded!";
    
    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
    
    while (!viewer.wasStopped()) {
        viewer.showCloud(cloud);
    }

}
