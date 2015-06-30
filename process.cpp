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

const bool DONT_GENERATE_MESH = false;

void printPercenrtage(const unsigned int processed, const unsigned int total){
    std::cout << processed << "/" << total << " done.\n";
}

int main(int argc, char** argv) {

    // if(argc != 5){
    //     std::cout << "usage:\n" << argv[0] << " x_pos y_pos z_pos OUT_NAME\n" 
    //                 << "\tx_pos\t\tdistance from kinect(as meter)\n\ty_pos\t\twidth from kinect (as meter)\n\tz_pos\t\twidth from kinect(as meter)\n"
    //                 << "\tOUT_NAME\tbase name for output files (file extension is optional)\n";
    //      return -1;
    // }
    if(argc != 2){
        std::cout << "usage:\n" << argv[0] << " OUT_NAME\n" 
                    << "\tOUT_NAME\tbase name for output files (file extension is optional)\n";
         return -1;
    }


    // output files
    std::stringstream pcd_out_ss, vtk_out_ss, ply_out_ss;
    pcd_out_ss << argv[1] << ".pcd";
    vtk_out_ss << argv[1] << ".vtk";
    ply_out_ss << argv[1] << ".ply";

    const std::string pcd_out_file(pcd_out_ss.str()),
                     vtk_out_file(vtk_out_ss.str()),
                     ply_out_file(ply_out_ss.str());

    // PointCloudRGBAPtr cloud_in(new PointCloudRGBA());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "loading files...\n";

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("concated.pcd", *final) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read the first .pcd file \n");
        return (-1);
    }

    
    printPercenrtage(1, 6);
    

    std::cout << "all files has read.\n";

    
#ifdef DEBUG_MODE
    std::cout << "cloud info:\th: " << final->height << " | w: " << final->width << std::endl;
#endif

    std::cout << "Cropping cloud...\n";
    
    BasicOperations cropper;

    // const float x_border_min = -11.5,
    //         x_border_max = 11.5;

    // const float y_border_min = -7,
    //         y_border_max = 7;

    // const float z_border_min = -6,
    //         z_border_max = 6;    

    const float x_border_min = -12,
            x_border_max = 12;

    const float y_border_min = -12,
            y_border_max = 12;

    const float z_border_min = -12,
            z_border_max = 12;

    cropper.setXMin(x_border_min);
    cropper.setXMax(x_border_max);

    cropper.setYMin(y_border_min);
    cropper.setYMax(y_border_max);

    cropper.setZMin(z_border_min);
    cropper.setZMax(z_border_max);

    cropper.cropCloud(final, cloud_out);
    std::cout << "cleaning useless points...\n";
    cropper.cleanNoP(cloud_out); // clean NoP points

    // cloud_out = final;


#ifdef DEBUG_MODE
    std::cout << "cloud info:\th: " << cloud_out->height << " | w: " << cloud_out->width << std::endl;
#endif
    
    // pcl::visualization::CloudViewer viewer("Crop Test");
    // while (!viewer.wasStopped()) {
    //     viewer.showCloud(cloud_out);
    // }

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->setBackgroundColor(0,0,0);
    // viewer->addCoordinateSystem(1.0);
    // viewer->addPointCloud(cloud_out);

    // while(!viewer->wasStopped()){
    //   viewer->spinOnce(100);
    //   boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }

    // restore original position
   

   /**** shiftCloud(cloud_out, -1 * x_mov, -1 * y_mov, -1 * z_mov);  */
    
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    cloud_out->height *= 480;
    cloud_out->width /= 480;
    cloud_out->is_dense = false;
    
   pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud (cloud_out);
   sor.setLeafSize (0.0005f, 0.0005f, 0.0005f);
   sor.filter (*cloud_filtered);

    std::cout << "saving to pcd file: " << pcd_out_file << std::endl;

    pcl::io::savePCDFile<PointXYZ>(pcd_out_file, *cloud_filtered); // as pcd file


#ifdef DEBUG_MODE
    std::cout << "size() --> " << cloud_out->size()
            << " w --> " << cloud_out->width << " h -->" << cloud_out->height << std::endl;
    
#endif

    if (!DONT_GENERATE_MESH) {

        // Normal estimation*
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud = cloud_filtered;
        
        std::cout << "meshing...\n";

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        
        std::cout << "calculating normals...\n";
        
        n.compute(*normals);
        //* normals should not contain the point normals + surface curvatures

        std::cout << "normals calculated\n";
        
        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
        //* cloud_with_normals = cloud + normals

        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);

        std::cout << "initializing mesh object\n";
        
        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        std::cout << "generating mesh...\n";
        
        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(0.025);

        // Set typical values for the parameters
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18); // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
        gp3.setNormalConsistency(false);

        std::cout << "exporting mesh...\n";
        
        // Get result
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

        // Additional vertex information
        std::vector<int> parts = gp3.getPartIDs();
        std::vector<int> states = gp3.getPointStates();

        std::cout << "saving VTK file...\n";

        pcl::io::saveVTKFile(vtk_out_file, triangles); //as mesh

        std::cout << "saving PLY file...\n";

        pcl::io::savePLYFile(ply_out_file, triangles); //as mesh 

    }

    std::cout << "done!\n";

    return 0;
}