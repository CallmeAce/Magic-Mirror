#ifndef MAGIC_MIRROR_H
#define MAGIC_MIRROR_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>



//  Mirror Mirror on the wall..... 
class Magic_Mirror
{  
  private:
      // parameters setting
       //   decide the range image getted from point cloud angular resolution 
      float angular_resolution;
      const float Roi_Range = 1.5;// default value
      //    decide the coordinate of range image
      pcl::RangeImage::CoordinateFrame R_coordinate_frame;

      //    parameters for pcl::RangeImage::createFromPointCloud
      bool setUnseenToMaxRange;
      float noise_level;
      float min_range;
      int border_size;
      
       

  public:

      typedef pcl::PointXYZ PointT;// define the point type
      pcl::PointCloud<PointT>::Ptr cloud_ptr;
      pcl::PointCloud<PointT>::Ptr cloud_downS;//outcome of downsampled cloud;
      pcl::PointCloud<PointT>::Ptr raw_object;

      Magic_Mirror (); //constractor  //     
      ~Magic_Mirror(); //disconstractor

      pcl::visualization::PCLVisualizer viewer;

//      pcl::visualization::CloudViewer viewer1;

  

// function 
 
           // callback to the Opennigrabber, getting data stream from kinect                    
      void cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud);
 
           // using octree algorithm compress the data
      void Octree_Compression (const pcl::PointCloud<PointT>::ConstPtr &cloud);

          // Filter the data and get the data needed
      void Filters (pcl::PointCloud<PointT>::Ptr & raw_data);
       
          // Data Segmentation
      void Segmentation(pcl::PointCloud<PointT>::Ptr & input_data,pcl::PointCloud<PointT>::Ptr & output_data);

          // point cloud clustering 
      void Clustering (pcl::PointCloud<PointT>::Ptr & raw_data,int cluster);   

          // read pcd file from disk
      void Read_Pcd_f ();
    
          // creat range_image from Pointcloud
      void Creat_Range_Img (const pcl::PointCloud<PointT>::ConstPtr &cloud);

          // extract border from range image
      void RImg_BorderExtra ();

         // visualization 
      void Magic_Visualizer ();



}




#endif
