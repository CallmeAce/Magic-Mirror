#include "Magic_Processing.h"
typedef pcl::PointXYZ PointT;
int main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;

	reader.read ("test0.pcd", *cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("test"));// important
        
	Magic_Processing magic;
    magic.Set_ROI(1,3);
    magic.Cloud_ROI(cloud,magic.m_cloud_roi);
    magic.Octree_Compression(magic.m_cloud_roi,magic.m_cloud_downS,0.01,0.01,0.01);
    magic.Ransac_ground_plane (magic.m_cloud_downS,magic.gr_coefficients);
    std::cout<<"ground_plane:  " <<magic.gr_coefficients<<std::endl;
    magic.BK_Filters(magic.m_cloud_downS,magic.wall_coefficient,magic.m_cloud_filtered,0.02);
    std::cout<<"roi"<< magic.m_cloud_roi->points.size()<<std::endl;
    std::cout<<"downs"<<magic.m_cloud_downS->points.size()<<std::endl;
    std::cout<<"filtered"<<magic.m_cloud_filtered->points.size()<<std::endl;
    magic.Clustering (magic.m_cloud_filtered,magic.m_cloud_cluster_points,3);//clustering
    magic.Projection (magic.m_cloud_cluster_points); 
    magic.rotation (magic.m_proj_vector,magic.m_rot_proj_vector);  
    pcl::PointCloud<pcl::PointXYZ>::Ptr t (new pcl::PointCloud<PointT>);
  //  magic.rotation (magic.m_proj_vector[0],t); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr v_case;
 
 //   pcl::PointCloud<pcl::PointXYZ>::Ptr v_case (n 
    for(unsigned int i = 0;i<3;i++)
    {	   
//		viewer->addPointCloud<PointT>(magic.m_cloud_filtered,"t");
        std::stringstream s1;
        std::stringstream s2;
        s1<<"v"<<i;
        s2<<"vv"<<i;

        v_case = magic.m_cloud_cluster_points[i].makeShared();// actually it makes a copy of pointcloud, it should be deleted manually         
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(magic.m_rot_proj_vector[i], 0, 255, 0);
		
	    viewer->addPointCloud<PointT>(v_case,s1.str());
        viewer->addPointCloud<PointT>(magic.m_rot_proj_vector[i],single_color,s2.str());
    }            
                
  //  viewer->resetCameraViewpoint("vv0");
    viewer->addCoordinateSystem (1.0);
    while(!viewer->wasStopped())
	{
		viewer->spinOnce (100);
	}



}
