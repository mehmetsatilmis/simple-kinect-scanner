#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include "rs232.h"

using namespace std; 

const string OUT_DIR = "./"; 

bool step(int cport_nr){
    unsigned char sendData = '1';
 
    SendByte(cport_nr, sendData);

    for(int wait=0;wait<10000000;wait++);
    
    return true;

}

class SimpleOpenNIViewer 
{ 
public: 
    SimpleOpenNIViewer () : viewer ("PCL Viewer"), portNum(0), baudRate(9600)
    { 
                frames_saved = 0; 
                save_one = false;
                OpenComport(portNum, baudRate);

    } 

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) 
    { 
                if (!viewer.wasStopped()) { 
                         viewer.showCloud (cloud); 

                //         if( save_one ) { 
                //                 save_one = false;
                                ++frames_saved;//
                                std::stringstream out; 
                                out << frames_saved; 
                                std::string name = OUT_DIR + "cloud" + out.str() + ".pcd"; 
                                pcl::io::savePCDFileASCII( name, *cloud );
                                step(portNum);
                                //step(portNum);
                                boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
                } 
    } 

    void run () 
    { 
                pcl::Grabber* interface = new pcl::OpenNIGrabber(); 

                boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
                        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1); 

                interface->registerCallback (f); 

                interface->start (); 

                char c; 

                while (!viewer.wasStopped()) 
                { 
                        //sleep (1); 

                        // c = getchar(); 
                        // if( c == 's' ) { 
                        //         cout << "Saving frame " << frames_saved << ".\n"; 
                        //         frames_saved++; 
                        //         save_one = true; 
                        // }

                        if(frames_saved >= 13)
                            break;
                } 

                interface->stop (); 
        } 

        pcl::visualization::CloudViewer viewer; 

        private: 
                int frames_saved; 
                bool save_one;
                int portNum, baudRate;

}; 

int main () 
{ 
    SimpleOpenNIViewer v; 
    v.run (); 
    std::cout << "scan done!\n";
    return 0; 
} 