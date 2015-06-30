#ifndef __INCLUDE_BASIC_OPS_H
#define __INCLUDE_BASIC_OPS_H

#include <limits>
#include <cmath>
#include <string>
#include <sstream>

#include <pcl/pcl_exports.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


#include "CommonTypes.h"

static const PointRGBA initRGBACloud (const float xPos, const float yPos, const float zPos){
    static PointRGBA point;
    point.x = xPos;
    point.y = yPos;
    point.z = zPos;
    return point;
}

static const PointXYZ initXYZCloud (const float xPos, const float yPos, const float zPos){
    static PointXYZ point;
    point.x = xPos;
    point.y = yPos;
    point.z = zPos;
    return point;
}
		
class BasicOperations {

public:

    const PointRGBA NoP_RGBA;
    const PointXYZ NoP_XYZ;
    const float NaN;


    BasicOperations() : NaN(std::numeric_limits <float>::quiet_NaN()), NoP_RGBA(initRGBACloud(NaN, NaN, NaN)), NoP_XYZ(initXYZCloud(NaN, NaN, NaN))
    {   
      // NoP.x = NoP.y = NoP.z = std::numeric_limits <float>::quiet_NaN();
    } // default constructor

  /**
   *  axis min and max values should be set before calling this method 
   *
   */
    void cropCloud(const PointCloudRGBAConstPtr cloud_in, PointCloudRGBAPtr cloud_out){
        if(NULL == cloud_out)
            cloud_out = PointCloudRGBAPtr(new PointCloudRGBA());

        const float x_min = .01f * _x_min;
        const float x_max = .01f * _x_max;
        const float y_min = .01f * _y_min;
        const float y_max = .01f * _y_max;
        const float z_min = .01f * _z_min;
        const float z_max = .01f * _z_max;

        cloud_out->reserve(cloud_in->size());

        for(PointCloudRGBA::const_iterator i = cloud_in->begin(); i != cloud_in->end(); ++i) {

            if( boost::math::isnan(i->x) || i->x <= x_min || i->x >= x_max || i->y <= y_min || i->y >= y_max || i->z <= z_min || i->z >= z_max)
                //PASS;
                cloud_out->push_back(NoP_RGBA);
            else
                cloud_out->push_back(*i);
        }

        cloud_out->width = cloud_in->width;
        cloud_out->height = cloud_in->height;
        cloud_out->is_dense = false;    

    }

    /**
	 *	axis min and max values should be set before calling this method 
	 *
	 */
	void cropCloud(const PointCloudXYZConstPtr cloud_in, PointCloudXYZPtr cloud_out){
		if(NULL == cloud_out)
			cloud_out = PointCloudXYZPtr(new PointCloudXYZ());

		const float x_min = .01f * _x_min;
       	const float x_max = .01f * _x_max;
       	const float y_min = .01f * _y_min;
       	const float y_max = .01f * _y_max;
       	const float z_min = .01f * _z_min;
       	const float z_max = .01f * _z_max;

       	cloud_out->reserve(cloud_in->size());

       	for(PointCloudXYZ::const_iterator i = cloud_in->begin(); i != cloud_in->end(); ++i) {

           	if( boost::math::isnan(i->x) || i->x <= x_min || i->x >= x_max || i->y <= y_min || i->y >= y_max || i->z <= z_min || i->z >= z_max)
              //dPASS;
             	cloud_out->push_back(NoP_XYZ);
           	else
             	cloud_out->push_back(*i);
       	}

       	cloud_out->width = cloud_in->width;
      	cloud_out->height = cloud_in->height;
      	cloud_out->is_dense = false;		

	}

    /**
     *
     * Cleans points marked as Not a Point in given point cloud4
     * return total number of deletions
     *
     */
    size_t cleanNoP(PointCloudRGBAPtr cloud_in){

        size_t count = 0;
        for(PointCloudRGBA::iterator i = cloud_in->begin(); i != cloud_in->end(); ++i) {

            if( boost::math::isnan(i->x) || boost::math::isnan(i->y) || boost::math::isnan(i->z) ){

                cloud_in->erase(i);
                ++count;
            }

        }

        return count;
    }

    /**
     *
     * Cleans points marked as Not a Point in given point cloud4
     * return total number of deletions
     *
     */
    size_t cleanNoP(PointCloudXYZPtr cloud_in){

        size_t count = 0;
        for(PointCloudXYZ::iterator i = cloud_in->begin(); i != cloud_in->end(); ++i) {

            if( boost::math::isnan(i->x) || boost::math::isnan(i->y) || boost::math::isnan(i->z) ){

                cloud_in->erase(i);
                ++count;
            }

        }

        return count;
    }

	inline void setXMin (const float x_min) {if (x_min < _x_max) _x_min = x_min;}
   	inline void setXMax (const float x_max) {if (x_max > _x_min) _x_max = x_max;}
   	inline void setYMin (const float y_min) {if (y_min < _y_max) _y_min = y_min;}
   	inline void setYMax (const float y_max) {if (y_max > _y_min) _y_max = y_max;}
   	inline void setZMin (const float z_min) {if (z_min < _z_max) _z_min = z_min;}
   	inline void setZMax (const float z_max) {if (z_max > _z_min) _z_max = z_max;}

   	inline float getXMin () const {return (_x_min);}
   	inline float getXMax () const {return (_x_max);}
   	inline float getYMin () const {return (_y_min);}
   	inline float getYMax () const {return (_y_max);}
   	inline float getZMin () const {return (_z_min);}
   	inline float getZMax () const {return (_z_max);}

private:
  
   // Clipping boundaries in centimeters
   float _x_min;
   float _x_max;
   float _y_min;
   float _y_max;
   float _z_min;
   float _z_max;
	
};

/**
 *
 *
 * Shifts given cloud by given amouns on axis
 * x_mov - shift amount on x axis
 * y_mov - shift amount on y axis
 * z_mov - shift amount on z axis
 *
 */
void shiftCloud(PointCloudRGBAPtr cloud_in, const float x_mov, const float y_mov, const float z_mov){

    const unsigned int width  = cloud_in->width;
    const unsigned int height = cloud_in->height;

    for(size_t i = 0; i < cloud_in->size(); ++i){
        cloud_in->points[i].x -= x_mov;
        cloud_in->points[i].y -= y_mov;
        cloud_in->points[i].z -= z_mov;
    }

    cloud_in->width = width;
    cloud_in->height = height;
    cloud_in->is_dense = false; 

}

/**
 *
 *
 * Shifts given cloud by given amouns on axis
 * x_mov - shift amount on x axis
 * y_mov - shift amount on y axis
 * z_mov - shift amount on z axis
 *
 */
void shiftCloud(PointCloudXYZPtr cloud_in, const float x_mov, const float y_mov, const float z_mov){

    const unsigned int width  = cloud_in->width;
    const unsigned int height = cloud_in->height;

    for(size_t i = 0; i < cloud_in->size(); ++i){
        cloud_in->points[i].x -= x_mov;
        cloud_in->points[i].y -= y_mov;
        cloud_in->points[i].z -= z_mov;
    }

    cloud_in->width = width;
    cloud_in->height = height;
    cloud_in->is_dense = false;  

}

const double RAD_PER_DEGREE = 0.0174532925;

double toRadian(const double degree){
    return degree * RAD_PER_DEGREE;
}

Eigen::Matrix4f generateTransformMatrixY(const double degree){
    double rad = toRadian(degree);
    Eigen::Matrix4f transform;
    
    transform <<  cos(rad), 0, sin(rad), 0,
                  0, 1, 0, 0,
                  (-1 * sin(rad)), 0, cos(rad), 0,
                  0,0,0,1;
    
    return transform;    
}

void showCloud(PointCloudXYZPtr cloud) {
    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
    
    while (!viewer.wasStopped()) {
        viewer.showCloud(cloud);
    }
}

void showCloud(PointCloudRGBAPtr cloud) {
    pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
    
    while (!viewer.wasStopped()) {
        viewer.showCloud(cloud);
    }
}


#endif