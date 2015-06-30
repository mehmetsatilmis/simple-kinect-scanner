#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cstdio>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/registration_visualizer.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "BasicOperations.h"

void printPercenrtage(const unsigned int processed, const unsigned int total){
    std::cout << processed << "/" << total << " done.\n";
}

int main(int argc, char** argv) {

    if(argc != 4){
        std::cout << "usage:\n" << argv[0] << " x_pos y_pos z_pos\n" 
                    << "\tx_pos\t\tdistance from kinect(as meter)\n\ty_pos\t\twidth from kinect (as meter)\n\tz_pos\t\twidth from kinect(as meter)\n"
                    << "\t\tmanipulated points will be saved in concated.pcd file\n";
         return -1;
    }

    // for shifting clouds
    const float  x_mov = std::atof(argv[1]),
                 y_mov = std::atof(argv[2]),
                 z_mov = std::atof(argv[3]);


    // output files
    // std::stringstream pcd_out_ss, vtk_out_ss, ply_out_ss;
    // pcd_out_ss << argv[4] << ".pcd";
    // vtk_out_ss << argv[4] << ".vtk";
    // ply_out_ss << argv[4] << ".ply";

    // const std::string pcd_out_file(pcd_out_ss.str()),
    //                  vtk_out_file(vtk_out_ss.str()),
    //                  ply_out_file(ply_out_ss.str());

    std::string pcd_out_file("concated.pcd");

    // PointCloudRGBAPtr cloud_in(new PointCloudRGBA());
    PointCloudXYZPtr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    const int inputSize = 12;

    pcl::PointCloud<pcl::PointXYZ> inputCloud[inputSize];

    std::cout << "loading files...\n";

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud1.pcd", inputCloud[0]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }

    printPercenrtage(1, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud2.pcd", inputCloud[1]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }

    printPercenrtage(2, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud3.pcd", inputCloud[2]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }

    printPercenrtage(3, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud4.pcd", inputCloud[3]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }
    
    printPercenrtage(4, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud5.pcd", inputCloud[4]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }
    
    printPercenrtage(5, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud6.pcd", inputCloud[5]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }
    
    printPercenrtage(6, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud7.pcd", inputCloud[6]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }

    printPercenrtage(7, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud8.pcd", inputCloud[7]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }

    printPercenrtage(8, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud9.pcd", inputCloud[8]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }

    printPercenrtage(9, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud10.pcd", inputCloud[9]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }
    
    printPercenrtage(10, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud11.pcd", inputCloud[10]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }
    
    printPercenrtage(11, inputSize);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud12.pcd", inputCloud[11]) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }
    
    printPercenrtage(12, inputSize);

    std::cout << "all files has read.\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputs[inputSize];

    for (int i = 0; i < inputSize; ++i) {
        inputs[i] = inputCloud[i].makeShared();
    }

#ifdef DEBUG_MODE
    std::cout << "cloud info:\th: " << inputs[0]->height << " | w: " << inputs[0]->width << std::endl;
    std::cout << "cloud info:\th: " << inputs[1]->height << " | w: " << inputs[1]->width << std::endl;
    std::cout << "cloud info:\th: " << inputs[2]->height << " | w: " << inputs[2]->width << std::endl;
    std::cout << "cloud info:\th: " << inputs[3]->height << " | w: " << inputs[3]->width << std::endl;
#endif

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2, cloud3, cloud4;

    std::cout << "transporting to center\n";

    // const float x_mov = -0.03,
    //         y_mov = 0.0,
    //         z_mov = 0.70;

    shiftCloud(inputs[0], x_mov, y_mov, z_mov);
    shiftCloud(inputs[1], x_mov, y_mov, z_mov);
    shiftCloud(inputs[2], x_mov, y_mov, z_mov);
    shiftCloud(inputs[3], x_mov, y_mov, z_mov);
    shiftCloud(inputs[4], x_mov, y_mov, z_mov);
    shiftCloud(inputs[5], x_mov, y_mov, z_mov);
    shiftCloud(inputs[6], x_mov, y_mov, z_mov);
    shiftCloud(inputs[7], x_mov, y_mov, z_mov);
    shiftCloud(inputs[8], x_mov, y_mov, z_mov);
    shiftCloud(inputs[9], x_mov, y_mov, z_mov);
    shiftCloud(inputs[10], x_mov, y_mov, z_mov);
    shiftCloud(inputs[11], x_mov, y_mov, z_mov);

#ifdef DEBUG_MODE
    std::cout << "cloud info:\th: " << inputs[0]->height << " | w: " << inputs[0]->width << std::endl;
    std::cout << "cloud info:\th: " << inputs[1]->height << " | w: " << inputs[1]->width << std::endl;
    std::cout << "cloud info:\th: " << inputs[2]->height << " | w: " << inputs[2]->width << std::endl;
    std::cout << "cloud info:\th: " << inputs[3]->height << " | w: " << inputs[3]->width << std::endl;
#endif

    std::cout << "rotating...\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud3_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud4_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud5_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud6_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud7_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud8_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud9_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud10_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud11_1(new pcl::PointCloud<pcl::PointXYZ>),
            cloud12_1(new pcl::PointCloud<pcl::PointXYZ>);

    transformPointCloud(*inputs[1], *cloud2_1, generateTransformMatrixY(30));
    transformPointCloud(*inputs[2], *cloud3_1, generateTransformMatrixY(60));
    transformPointCloud(*inputs[3], *cloud4_1, generateTransformMatrixY(90));
    transformPointCloud(*inputs[4], *cloud5_1, generateTransformMatrixY(120));
    transformPointCloud(*inputs[5], *cloud6_1, generateTransformMatrixY(150));
    transformPointCloud(*inputs[6], *cloud7_1, generateTransformMatrixY(180));
    transformPointCloud(*inputs[7], *cloud8_1, generateTransformMatrixY(210));
    transformPointCloud(*inputs[8], *cloud9_1, generateTransformMatrixY(240));
    transformPointCloud(*inputs[9], *cloud10_1, generateTransformMatrixY(270));
    transformPointCloud(*inputs[10], *cloud11_1, generateTransformMatrixY(300));
    transformPointCloud(*inputs[11], *cloud12_1, generateTransformMatrixY(330));

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_2 (new pcl::PointCloud<pcl::PointXYZ>),
            cloud3_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud4_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud5_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud6_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud7_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud8_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud9_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud10_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud11_2(new pcl::PointCloud<pcl::PointXYZ>),
            cloud12_2(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::VoxelGrid<pcl::PointXYZ> sor1;
    sor1.setLeafSize (0.0005f, 0.0005f, 0.0005f);
    sor1.setInputCloud (cloud2_1);    
    sor1.filter (*cloud2_2);

    sor1.setInputCloud (cloud3_1);    
    sor1.filter (*cloud3_2);

    sor1.setInputCloud (cloud4_1);    
    sor1.filter (*cloud4_2);

    sor1.setInputCloud (cloud5_1);    
    sor1.filter (*cloud5_2);

    sor1.setInputCloud (cloud6_1);    
    sor1.filter (*cloud6_2);

    sor1.setInputCloud (cloud7_1);    
    sor1.filter (*cloud7_2);

    sor1.setInputCloud (cloud8_1);    
    sor1.filter (*cloud8_2);

    sor1.setInputCloud (cloud9_1);    
    sor1.filter (*cloud9_2);

    sor1.setInputCloud (cloud10_1);    
    sor1.filter (*cloud10_2);

    sor1.setInputCloud (cloud11_1);    
    sor1.filter (*cloud11_2);

    sor1.setInputCloud (cloud12_1);    
    sor1.filter (*cloud12_2);

#ifdef DEBUG_MODE
    std::cout << "cloud info:\th: " << cloud2_1->height << " | w: " << cloud2_1->width << std::endl;
    std::cout << "cloud info:\th: " << cloud3_1->height << " | w: " << cloud3_1->width << std::endl;
    std::cout << "cloud info:\th: " << cloud4_1->height << " | w: " << cloud4_1->width << std::endl;
#endif

    std::cout << "conatenating...\n";

    // pcl::PointCloud<pcl::PointXYZRGBA> final = cloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final = inputs[0];
    *final += *cloud2_2;
    *final += *cloud3_2;
    *final += *cloud4_2;
    *final += *cloud5_2;
    *final += *cloud6_2;
    *final += *cloud7_2;
    *final += *cloud8_2;
    *final += *cloud9_2;
    *final += *cloud10_2;
    *final += *cloud11_2;
    *final += *cloud12_2;

    // pcl::visualization::CloudViewer viewer("Crop Test");
    // while (!viewer.wasStopped()) {
    //     viewer.showCloud(final);
    // }  

#ifdef DEBUG_MODE
    std::cout << "cloud info:\th: " << final->height << " | w: " << final->width << std::endl;
#endif

    final->height *= 480;
    final->width /= 480;
    final->is_dense = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (final);
    sor.setLeafSize (0.0005f, 0.0005f, 0.0005f);
    sor.filter (*cloud_filtered);

    std::cout << "saving to pcd file...\n";

    pcl::io::savePCDFile<PointXYZ>(pcd_out_file, *cloud_filtered); // as pcd file

    return 0;
}