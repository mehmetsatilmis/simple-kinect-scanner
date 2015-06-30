#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>


class KinectReaderXYZ {

public:
    KinectReaderXYZ() : _interface(new pcl::OpenNIGrabber())
    { /* intentionally left blank */ }

    ~KinectReaderXYZ(){
        delete _interface;
    }

    void init(void){
        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = 
            boost::bind (&KinectReaderXYZ::cloud_cb_, this, _1); 

        _interface->registerCallback (f);
        _interface->start();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCurrent(void){
        return _cloud.makeShared();
    }

    void stop(void){
        _interface->stop(); 
    }

private:
    pcl::Grabber* _interface;
    pcl::PointCloud<pcl::PointXYZ> _cloud;

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) 
    { 
        _cloud = *cloud;
    } 

};

// class SimpleOpenNIViewer 
// { 
// public: 
//     SimpleOpenNIViewer () : viewer ("PCL Viewer") 
//     { 
//                 frames_saved = 0; 
//                 save_one = false; 
//     } 

//     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) 
//     { 
//                 if (!viewer.wasStopped()) { 
//                         viewer.showCloud (cloud); 

//                         if( save_one ) { 
//                                 save_one = false; 
//                                 std::stringstream out; 
//                                 out << frames_saved; 
//                                 std::string name = OUT_DIR + "cloud" + out.str() + ".pcd"; 
//                                 pcl::io::savePCDFileASCII( name, *cloud ); 
//                         } 
//                 } 
//     } 

//     void run () 
//     { 
//                 pcl::Grabber* interface = new pcl::OpenNIGrabber(); 

//                 boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
//                         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1); 

//                 interface->registerCallback (f); 

//                 interface->start (); 

//                 char c; 

//                 while (!viewer.wasStopped()) 
//                 { 
//                         //sleep (1); 

//                         c = getchar(); 
//                         if( c == 's' ) { 
//                                 cout << "Saving frame " << frames_saved << ".\n"; 
//                                 frames_saved++; 
//                                 save_one = true; 
//                         } 
//                 } 

//                 interface->stop (); 
//         } 

//         pcl::visualization::CloudViewer viewer; 

//         private: 
//                 int frames_saved; 
//                 bool save_one; 

// }; 