#ifndef POINTXYZIR_H
#define POINTXYZIR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

/*
    * A point cloud type that has "ring" channel
    */

struct EIGEN_ALIGN16 PointXYZIR{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static inline PointXYZIR make(float x,float y,float z,float intensity,uint16_t ring){
       return {x,y,z,0.0,intensity,ring};
}
};

//clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
(float, x, x)
(float, y, y)
(float, z, z) 
(float, intensity, intensity)
(uint16_t, ring, ring)
)

typedef pcl::PointXYZI  PointType;

#endif
