#include "Magic_Mirror.h"     




Magic_Mirror::Magic_Mirror () : viewer ("3D Viewer") //constractor initialization
{ 
  angular_resolution  = pcl::deg2rad(2.0f);
  R_coordinate_frame  = pcl::RangeImage::CAMERA_FRAME;
  setUnseenToMaxRange = true;
  noise_level         = 0.0;
  min_range           = 0.0f;
  border_size         = 1;
  viewer.setBackgroundColor (1,1,1);
  viewer.addCoordinateSystem (1.0f);

}

//--------------------main callback function----------------------------------
Magic_Mirror::cloud_cb_(const pcl::PointCloud<PointT>::ConstPtr &cloud)
{
  if (!viewer.wasStopped())
  viewer.showCloud(cloud);
  
}


//------------------------Octree Compression----------------------------------
Magic_Mirror:: Octree_Compression (const pcl::PointCloud<PointT>::ConstPtr & cloud)
{
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud) ;
  vg.setLeafSize (0.01f, 0.01f, 0.01f);// 1 cubic cm 
  vg.filter (*cloud_downS);
  std::cout << "PointCloud after downsampleing has" <<cloud_downS->points.size () << "data points." <<std::endl;

}


//--------------------Filter function-------------------------------------
Magic_Mirror:: Filters (pcl::PointCloud<PointT>::Ptr & raw_data)
{
  pcl::SACSegmentation<PointT> seg;//creat segmentation object;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>());//??
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);//sample consensus plane model
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (80);
  seg.setDistanceThreshold (0.02);
  int i = 0, num_points = (int) cloud_filtered -> points.size ();
  while (raw_data->points.size() >0.3*num_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (raw_data);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0 ) 
    {
      std::cout << "can not fit the plane model to the given data" <<std::endl;
      break;
    }      
    
   // Extract the planar inliers from the input cloud
    pcl::ExtratIndices<PointT> extract;
    extract.setInputCloud 


  }


}

  
//-----------------Segmentation Function-------------------------
Magic_Mirror::Segmentation (pcl::PointCloud<PointT>::Ptr & input_data, pcl::PointCloud<PointT>::Ptr & output_data)
{  
    
  pcl::SACSegmentation<PointT> seg;//creat segmentation object;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients_Temp (new pcl::ModelCoefficients);//coefficients template
  coefficients_Temp->values.push_back(0);
  coefficients_Temp->values.push_back(1);
  coefficients_Temp->values.push_back(0);
  coefficients_Temp->values.push_back(0);
  
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);//sample consensus plane model
  seg.setMethodType (pcl::SAC_RANSAC);
  // I know.....this is ugly......but I am not confident enough to re-write the SACsegmentation class
  while(*coefficients
  seg.setMaxIterations (80);
  seg.setDistanceThreshold (0.02);
  int i = 0, num_points = (int) cloud_filtered -> points.size ();
 



  while (raw_data->points.size() >0.3*num_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (raw_data);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0 ) 
    {
      std::cout << "can not fit the plane model to the given data" <<std::endl;
      break;
    }      
    
   // Extract the planar inliers from the input cloud
    pcl::ExtratIndices<PointT> extract;
    extract.setInputCloud 

  }

}



