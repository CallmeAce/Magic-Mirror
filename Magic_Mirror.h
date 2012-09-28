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
#include "Magic_Processing.h"
#include "map/Magic_Mapping.h"
#include <pcl/common/common_headers.h>

//  Mirror Mirror on the wall..... 
class Magic_Mirror
{  
	private:

    static const int _Cluster_NUM = 5;              

	public:
// Variables
		
			typedef pcl::PointXYZ PointT;// define the point type
			pcl::PointCloud<PointT>::Ptr m_cloud_1;// default outcome 1
			pcl::PointCloud<PointT>::Ptr m_cloud_2;// default outcome 2
			pcl::PointCloud<PointT>::Ptr m_cloud_3;// default outcome 2
		    float m_inst_thresh; // for thresholding the map
			float m_thresh;// for thresholding the map
			std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > m_objects_vector;// outcomes from 	

			Magic_Mirror (); //constractor  //     
			~Magic_Mirror(); //disconstractor

			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
			// pcl::visualization::CloudViewer viewer1;

  

// Methods

			// callback to the Opennigrabber, getting data stream from kinect                    
			void cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr &cloud);

// TODO
			void Map_updating ();// to be finished (call the mapping class)
// TODO 
			void Magic_Visualizer ();// to be finished (call the visualization class)
            
            void run (); // The main structure of the vision system


};




#endif
