#ifndef POINTXYZIRPYT_H
#define POINTXYZIRPYT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>


/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */

struct EIGEN_ALIGN16 PointXYZIRPYT{
    PCL_ADD_POINT4D;
    float intensity;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static inline PointXYZIRPYT make(float x,float y,float z,float intensity,
                                     float roll,float pitch,float yaw,double time){
        return {x,y,z,0.0,intensity,roll,pitch,yaw,time};
}

};
//clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(float, roll, roll)
(float, pitch, pitch)
(float, yaw, yaw)
(double, time, time)
)
//clang-format on

typedef PointXYZIRPYT  PointTypePose;
#endif
